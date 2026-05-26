# ================================================================
# QSort ReID Backbone — Layer 7
# Plugs into qsort_full.py as a fifth cost-matrix term.
# Author: Deepak Pandey / extension for QSort framework
#
# WHAT THIS FILE CONTAINS:
#   1. QSortReIDBackbone  — MobileNetV3-small → 128-dim embedding
#   2. EmbeddingGallery   — per-track rolling gallery + cosine match
#   3. ReIDCostLayer      — computes cost term for the matrix
#   4. Training utilities — triplet + CE loss, full training loop
#   5. Integration guide  — how to plug into qsort_full.py
#
# USAGE (in your YOLO integration script):
#   from qsort_reid import QSortReIDBackbone, ReIDCostLayer
#   reid = QSortReIDBackbone(weights_path='qsort_reid.pth')  # or None for ImageNet init
#   # Each frame, after YOLO:
#   crops = [frame[y1:y2, x1:x2] for (x1,y1,x2,y2,_) in dets]
#   embeddings = reid.extract(crops)   # shape (N, 128)
#   # Pass embeddings into tracker.update(dets, embeddings)
# ================================================================

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights
from collections import deque
from typing import Optional, List
import cv2

# ----------------------------------------------------------------
# REID FINE-TUNING PARAMETERS
# ----------------------------------------------------------------
REID_EMB_DIM      = 128      # embedding dimension
REID_CROP_H       = 128      # crop height fed to backbone
REID_CROP_W       = 64       # crop width fed to backbone
REID_GALLERY_SIZE = 10       # embeddings kept per track
REID_DIST_MAX     = 250      # px — skip ReID for far detections
REID_ACTIVATE_MISSED = 0     # apply ReID even on frame-0 (use gallery mean)
REID_WEIGHT       = 0.35     # cost matrix weight for ReID term
REID_GATE_SIM     = 0.15     # below this similarity → hard reject (cost=1e6)

# Training hyperparameters
TRAIN_LR          = 0.01
TRAIN_EPOCHS      = 60
TRAIN_BATCH       = 64
TRAIN_MARGIN      = 0.3      # triplet loss margin
TRAIN_CE_WEIGHT   = 1.0      # cross-entropy weight
TRAIN_TRIPLET_W   = 0.8      # triplet loss weight


# ================================================================
# 1. BACKBONE — MobileNetV3-small + 128-dim ReID head
# ================================================================
class QSortReIDBackbone(nn.Module):
    """
    Lightweight ReID backbone for fish appearance embeddings.

    Architecture:
        MobileNetV3-small (features only, no classifier)
        → AdaptiveAvgPool2d(1,1)
        → Flatten
        → Linear(576, 256) → BN → ReLU → Dropout(0.3)
        → Linear(256, 128)
        → L2-normalise

    Input:  (N, 3, 128, 64) float32 tensor, values in [0,1]
    Output: (N, 128) L2-normalised float32 tensor

    For inference from numpy BGR crops:
        embeddings = reid.extract(list_of_bgr_crops)
    """

    def __init__(self,
                 weights_path: Optional[str] = None,
                 pretrained_imagenet: bool = True,
                 device: str = 'cpu'):
        super().__init__()

        self.device = torch.device(device)

        # Backbone — MobileNetV3-small feature extractor
        if pretrained_imagenet and weights_path is None:
            try:
                base = mobilenet_v3_small(
                    weights=MobileNet_V3_Small_Weights.IMAGENET1K_V1)
            except Exception:
                # No internet access in this environment — use random init
                # In production, download weights once and pass via weights_path
                print('[ReID] ImageNet weights unavailable — using random init')
                base = mobilenet_v3_small(weights=None)
        else:
            base = mobilenet_v3_small(weights=None)

        # Strip the classifier, keep only feature layers
        # MobileNetV3-small features output: (N, 576, H, W)
        self.features = base.features

        # ReID head
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(576, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(256, REID_EMB_DIM),
        )

        # Classification head — only used during training (CE loss)
        # num_classes set at training time via set_num_classes()
        self.classifier = None

        self.to(self.device)

        # Load trained weights if provided
        if weights_path is not None:
            self.load_weights(weights_path)

        # Preprocessing transform (ImageNet normalisation)
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((REID_CROP_H, REID_CROP_W)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]),
        ])

    def forward(self, x):
        """Forward pass — returns L2-normalised embeddings."""
        x = self.features(x)
        x = self.pool(x)
        x = self.head(x)
        x = F.normalize(x, p=2, dim=1)   # L2 normalise → unit sphere
        return x

    def forward_with_logits(self, x):
        """Forward pass for training — returns (embedding, logits)."""
        x = self.features(x)
        x = self.pool(x)
        emb = self.head(x)
        emb_norm = F.normalize(emb, p=2, dim=1)
        logits = self.classifier(emb) if self.classifier else None
        return emb_norm, logits

    def set_num_classes(self, n: int):
        """Call before training to attach classification head."""
        self.classifier = nn.Linear(REID_EMB_DIM, n).to(self.device)

    # ----------------------------------------------------------------
    # Inference helper — accepts raw BGR numpy crops
    # ----------------------------------------------------------------
    @torch.no_grad()
    def extract(self, crops: List[np.ndarray]) -> np.ndarray:
        """
        Extract embeddings from a list of BGR numpy crops.

        crops: list of (H, W, 3) uint8 BGR arrays (from cv2/YOLO)
        returns: (N, 128) float32 numpy array
        """
        if not crops:
            return np.empty((0, REID_EMB_DIM), dtype=np.float32)

        self.eval()
        tensors = []
        for crop in crops:
            if crop is None or crop.size == 0:
                tensors.append(torch.zeros(3, REID_CROP_H, REID_CROP_W))
                continue
            # BGR → RGB
            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            tensors.append(self.transform(rgb))

        batch = torch.stack(tensors).to(self.device)
        embs  = self(batch)
        return embs.cpu().numpy()

    def load_weights(self, path: str):
        """Load trained ReID weights."""
        state = torch.load(path, map_location=self.device)
        # Accept full checkpoint dict or raw state dict
        if 'model_state' in state:
            state = state['model_state']
        missing, unexpected = self.load_state_dict(state, strict=False)
        if missing:
            print(f'[ReID] Missing keys (expected on new head): {missing[:3]}')
        print(f'[ReID] Loaded weights from {path}')

    def save_weights(self, path: str, epoch: int = 0, meta: dict = None):
        torch.save({
            'model_state': self.state_dict(),
            'epoch': epoch,
            'meta': meta or {},
        }, path)
        print(f'[ReID] Saved weights → {path}')


# ================================================================
# 2. EMBEDDING GALLERY — per-track rolling store
# ================================================================
class EmbeddingGallery:
    """
    Maintains a rolling gallery of embeddings for one track.
    Matches new embeddings against the gallery mean.
    """

    def __init__(self, maxlen: int = REID_GALLERY_SIZE):
        self.gallery = deque(maxlen=maxlen)
        self._mean: Optional[np.ndarray] = None

    def update(self, emb: np.ndarray):
        """Add a new embedding to the gallery."""
        self.gallery.append(emb.astype(np.float32))
        self._mean = None   # invalidate cache

    @property
    def mean(self) -> Optional[np.ndarray]:
        """L2-normalised mean of gallery embeddings."""
        if not self.gallery:
            return None
        if self._mean is None:
            m = np.stack(self.gallery).mean(axis=0)
            norm = np.linalg.norm(m) + 1e-6
            self._mean = m / norm
        return self._mean

    def cosine_similarity(self, emb: np.ndarray) -> float:
        """Cosine similarity between emb and gallery mean. Range [−1, 1]."""
        if self.mean is None:
            return 0.0
        return float(np.dot(self.mean, emb))

    def reid_cost(self, emb: np.ndarray) -> float:
        """Cost term: 0 = perfect match, 2 = opposite (range [0, 2])."""
        return 1.0 - self.cosine_similarity(emb)

    def __len__(self):
        return len(self.gallery)


# ================================================================
# 3. REID COST LAYER — plugs into build_cost_matrix
# ================================================================
class ReIDCostLayer:
    """
    Computes the ReID contribution to the association cost matrix.

    Usage:
        reid_layer = ReIDCostLayer()

        # In build_cost_matrix, after computing other costs:
        reid_cost = reid_layer.cost(track, det_emb, dist)
        C[i, j] += reid_cost
    """

    def cost(self,
             gallery: EmbeddingGallery,
             det_emb: Optional[np.ndarray],
             dist: float,
             track_age: int,
             track_missed: int) -> float:
        """
        Returns ReID cost contribution for one (track, detection) pair.

        gallery    : track's EmbeddingGallery
        det_emb    : 128-dim embedding of the detection (or None if no backbone)
        dist       : Euclidean distance between track and detection centres
        track_age  : how many frames old the track is
        track_missed: how many frames the track has been missed
        """
        # Skip if ReID not active or no embeddings available
        if det_emb is None or len(gallery) == 0:
            return 0.0
        if dist > REID_DIST_MAX:
            return 0.0
        if track_age < 2:
            return 0.0

        raw_cost = gallery.reid_cost(det_emb)  # [0, 2]

        # Hard gate: if similarity below floor, return 1e6 (forces reject)
        sim = 1.0 - raw_cost
        if sim < REID_GATE_SIM and track_missed > 3:
            return 1e6

        return REID_WEIGHT * raw_cost


# ================================================================
# 4. TRAINING UTILITIES
# ================================================================
class TripletLoss(nn.Module):
    """Batch-hard triplet loss with squared Euclidean distance."""

    def __init__(self, margin: float = TRAIN_MARGIN):
        super().__init__()
        self.margin = margin

    def forward(self, embs: torch.Tensor, labels: torch.Tensor):
        # Pairwise L2 distance matrix
        n = embs.size(0)
        dist = torch.cdist(embs, embs, p=2)  # (N, N)

        # Mask same/different label pairs
        label_eq  = labels.unsqueeze(0) == labels.unsqueeze(1)  # (N,N)
        label_neq = ~label_eq

        # Batch-hard: hardest positive and hardest negative per anchor
        # Positives: same label, mask diagonal
        dist_ap = (dist * label_eq.float())
        dist_ap[~label_eq] = 0
        dist_ap, _ = dist_ap.max(dim=1)

        # Negatives: different label
        dist_an = dist.clone()
        dist_an[label_eq] = 1e9
        dist_an, _ = dist_an.min(dim=1)

        loss = F.relu(dist_ap - dist_an + self.margin).mean()
        return loss


class FishReIDDataset(torch.utils.data.Dataset):
    """
    Dataset for training ReID from bbox crops.

    Directory structure expected:
        data_dir/
            fish_001/
                frame_0042.jpg
                frame_0087.jpg
                ...
            fish_002/
                ...

    Each subdirectory is one fish identity.
    Crops should be extracted from your tracked video using your
    existing QSort tracker output (use confirmed tracks only).
    """

    def __init__(self, data_dir: str, augment: bool = True):
        import os
        self.samples = []   # list of (path, label_int)
        self.label_map = {}
        self.transform = self._build_transform(augment)

        ids = sorted(os.listdir(data_dir))
        for idx, fish_id in enumerate(ids):
            fish_dir = os.path.join(data_dir, fish_id)
            if not os.path.isdir(fish_dir):
                continue
            self.label_map[fish_id] = idx
            for fname in os.listdir(fish_dir):
                if fname.lower().endswith(('.jpg', '.jpeg', '.png')):
                    self.samples.append((os.path.join(fish_dir, fname), idx))

        self.num_classes = len(self.label_map)
        print(f'[ReID Dataset] {len(self.samples)} crops, '
              f'{self.num_classes} identities')

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        path, label = self.samples[idx]
        img = cv2.imread(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return self.transform(img), label

    def _build_transform(self, augment: bool):
        ops = [transforms.ToPILImage(),
               transforms.Resize((REID_CROP_H, REID_CROP_W))]
        if augment:
            ops += [
                transforms.RandomHorizontalFlip(),
                transforms.ColorJitter(
                    brightness=0.3, contrast=0.3,
                    saturation=0.2, hue=0.05),
                transforms.RandomAffine(
                    degrees=8, translate=(0.05, 0.1)),
            ]
        ops += [transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225])]
        return transforms.Compose(ops)


def train_reid(
        data_dir: str,
        save_path: str = 'qsort_reid.pth',
        epochs: int = TRAIN_EPOCHS,
        lr: float = TRAIN_LR,
        batch_size: int = TRAIN_BATCH,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu',
):
    """
    Full training loop for the QSort ReID backbone.

    Args:
        data_dir  : path to dataset directory (see FishReIDDataset)
        save_path : where to save the trained weights
        epochs    : training epochs
        lr        : initial learning rate (cosine decay used)
        batch_size: batch size
        device    : 'cuda' or 'cpu'

    Returns:
        Trained QSortReIDBackbone

    Example:
        model = train_reid('data/fish_crops/', 'qsort_reid.pth', epochs=60)
    """
    dataset = FishReIDDataset(data_dir, augment=True)
    loader  = torch.utils.data.DataLoader(
        dataset, batch_size=batch_size, shuffle=True,
        num_workers=4, drop_last=True, pin_memory=True)

    model = QSortReIDBackbone(
        pretrained_imagenet=True, device=device)
    model.set_num_classes(dataset.num_classes)
    model.to(device)

    triplet_loss = TripletLoss(margin=TRAIN_MARGIN)
    ce_loss = nn.CrossEntropyLoss(label_smoothing=0.1)

    # Separate LR for backbone (lower) vs head (higher)
    optimizer = torch.optim.SGD([
        {'params': model.features.parameters(), 'lr': lr * 0.1},
        {'params': model.head.parameters(),     'lr': lr},
        {'params': model.classifier.parameters(),'lr': lr},
    ], momentum=0.9, weight_decay=5e-4, nesterov=True)

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=epochs, eta_min=1e-5)

    best_loss = float('inf')
    for epoch in range(1, epochs + 1):
        model.train()
        total_loss = 0.0
        for imgs, labels in loader:
            imgs   = imgs.to(device)
            labels = labels.to(device)

            embs, logits = model.forward_with_logits(imgs)

            loss_tri = triplet_loss(embs, labels)
            loss_ce  = ce_loss(logits, labels)
            loss     = TRAIN_CE_WEIGHT * loss_ce + TRAIN_TRIPLET_W * loss_tri

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            total_loss += loss.item()

        scheduler.step()
        avg = total_loss / max(len(loader), 1)
        if epoch % 10 == 0 or epoch == 1:
            print(f'  Epoch {epoch:3d}/{epochs} | loss={avg:.4f} '
                  f'| lr={scheduler.get_last_lr()[0]:.5f}')
        if avg < best_loss:
            best_loss = avg
            model.save_weights(save_path, epoch=epoch,
                               meta={'loss': avg, 'n_ids': dataset.num_classes})

    print(f'Training complete. Best loss: {best_loss:.4f}')
    return model


# ================================================================
# 5. INTEGRATION GUIDE
# ================================================================
INTEGRATION_GUIDE = """
HOW TO ADD REID TO qsort_full.py
=================================

Step 1 — Add to imports and params at top of qsort_full.py:
------------------------------------------------------------
    from qsort_reid import (QSortReIDBackbone, EmbeddingGallery,
                            ReIDCostLayer, REID_WEIGHT, REID_DIST_MAX)
    USE_REID = True                     # ablation toggle
    REID_WEIGHTS = 'qsort_reid.pth'    # None = ImageNet init only

Step 2 — Add gallery to QSortTrack.__init__:
--------------------------------------------
    # After existing bookkeeping lines:
    self.reid_gallery = EmbeddingGallery()

Step 3 — Update gallery in QSortTrack.collapse():
--------------------------------------------------
    # At end of collapse(), add:
    def collapse(self, det, emb=None):
        ...existing code...
        if emb is not None:
            self.reid_gallery.update(emb)

Step 4 — Update build_cost_matrix signature:
--------------------------------------------
    def build_cost_matrix(tracks, detections, det_embeddings=None):
        ...
        reid_layer = ReIDCostLayer()
        for i, T in enumerate(tracks):
            for j, det in enumerate(detections):
                ...existing costs...
                # ReID cost (L7)
                if USE_REID and det_embeddings is not None:
                    det_emb = det_embeddings[j] if j < len(det_embeddings) else None
                    cost_reid = reid_layer.cost(
                        T.reid_gallery, det_emb, dist,
                        T.age, T.missed)
                    if cost_reid >= 1e5:
                        C[i, j] = 1e6   # hard reject
                        continue
                else:
                    cost_reid = 0.0
                C[i, j] = cost_dist + cost_boltz + cost_dir + cost_tensor + cost_reid + cost_iou

Step 5 — Update QSortPhysicsTracker.update() signature:
--------------------------------------------------------
    def update(self, dets, embeddings=None):
        ...
        C = build_cost_matrix(self.tracks, dets, embeddings)
        ...
        for r, c in zip(rows, cols):
            if C[r, c] < 1e5:
                emb = embeddings[c] if embeddings is not None else None
                self.tracks[r].collapse(dets[c], emb=emb)

Step 6 — In your YOLO integration script:
------------------------------------------
    from qsort_full import QSortPhysicsTracker
    from qsort_reid import QSortReIDBackbone

    reid   = QSortReIDBackbone(weights_path='qsort_reid.pth')
    tracker = QSortPhysicsTracker()

    cap = cv2.VideoCapture('barramundi.mp4')
    while True:
        ret, frame = cap.read()
        if not ret: break

        # YOLO detections — shape (N, 5): x1,y1,x2,y2,conf
        dets = yolo.detect(frame)

        # Extract crops and embeddings
        crops = [frame[int(d[1]):int(d[3]), int(d[0]):int(d[2])]
                 for d in dets]
        embeddings = reid.extract(crops)   # (N, 128) numpy

        # Track
        out = tracker.update(dets, embeddings=embeddings)

        # out: (M, 20) rows = x1,y1,x2,y2,id,vx,vy,...,R,thA,thB,thC

Step 7 — Training your ReID model:
------------------------------------
    # First collect crops from a manually-annotated short clip:
    #   mkdir -p data/fish_crops/fish_001 data/fish_crops/fish_002 ...
    #   Then save bbox crops per identity.
    #
    # Then train:
    from qsort_reid import train_reid
    model = train_reid('data/fish_crops/', 'qsort_reid.pth', epochs=60)
    #
    # Even 30-50 identities × 50 crops each = 2000 images is enough
    # to get a useful embedding space for barramundi.
"""


# ================================================================
# QUICK SANITY CHECK
# ================================================================
if __name__ == '__main__':
    print('QSort ReID Backbone — sanity check')
    print()

    # Build model
    model = QSortReIDBackbone(pretrained_imagenet=True)
    print(f'Backbone params : {sum(p.numel() for p in model.parameters()):,}')
    print(f'Embedding dim   : {REID_EMB_DIM}')
    print(f'Crop size       : {REID_CROP_H}×{REID_CROP_W}')
    print()

    # Fake crops (as if from cv2)
    fake_crops = [
        np.random.randint(0, 255, (80, 50, 3), dtype=np.uint8)
        for _ in range(4)
    ]
    embs = model.extract(fake_crops)
    print(f'Extracted embeddings shape : {embs.shape}')
    print(f'L2 norms (should all be 1) : {np.linalg.norm(embs, axis=1).round(4)}')
    print()

    # Gallery test
    gallery = EmbeddingGallery()
    gallery.update(embs[0])
    gallery.update(embs[1])

    sim_same  = gallery.cosine_similarity(embs[0])  # same fish
    sim_diff  = gallery.cosine_similarity(embs[2])  # random fish
    print(f'Cosine sim (same embedding)   : {sim_same:.4f}  (should be ~1.0 pre-training)')
    print(f'Cosine sim (random embedding) : {sim_diff:.4f}')
    print()

    # Cost layer
    reid_layer = ReIDCostLayer()
    cost = reid_layer.cost(gallery, embs[2], dist=50.0,
                           track_age=5, track_missed=0)
    print(f'ReID cost for random embedding : {cost:.4f}')
    print()
    print('Integration guide:')
    print(INTEGRATION_GUIDE[:600], '...')
    print()
    print('Run train_reid() with your fish crop dataset to get trained weights.')
