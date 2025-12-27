#!/usr/bin/env python3
"""
Model Training Script

Trains the PointNet classifier on synthetic dataset.
Saves trained model weights for real-time classification.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torch.nn.functional as F
from pathlib import Path
import json
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
import time


class PointNetClassifier(nn.Module):
    """
    Simplified PointNet architecture for point cloud classification.
    Must match the architecture in ml_object_classifier.py!
    """
    
    def __init__(self, num_classes=4):
        super(PointNetClassifier, self).__init__()
        
        # Shared MLP layers
        self.conv1 = nn.Conv1d(3, 64, 1)
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.conv3 = nn.Conv1d(128, 256, 1)
        
        # Classification head
        self.fc1 = nn.Linear(256, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, num_classes)
        
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(256)
        self.bn4 = nn.BatchNorm1d(128)
        self.bn5 = nn.BatchNorm1d(64)
        
        self.dropout = nn.Dropout(p=0.3)
        
    def forward(self, x):
        # x shape: (batch_size, num_points, 3)
        # Transpose to (batch_size, 3, num_points) for Conv1d
        x = x.transpose(2, 1)
        
        # Shared MLP
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        
        # Global max pooling
        x = torch.max(x, 2)[0]  # (batch_size, 256)
        
        # Classification MLP
        x = F.relu(self.bn4(self.fc1(x)))
        x = self.dropout(x)
        x = F.relu(self.bn5(self.fc2(x)))
        x = self.dropout(x)
        x = self.fc3(x)
        
        return x


class PointCloudDataset(Dataset):
    """Dataset for point cloud samples"""
    
    def __init__(self, data, labels):
        self.data = torch.FloatTensor(data)
        self.labels = torch.LongTensor(labels)
    
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        return self.data[idx], self.labels[idx]


def load_dataset(dataset_path):
    """Load all samples from dataset directory"""
    dataset_path = Path(dataset_path)
    
    class_names = ['sphere', 'box', 'cylinder', 'cone']
    class_to_idx = {name: idx for idx, name in enumerate(class_names)}
    
    all_data = []
    all_labels = []
    
    print("Loading dataset...")
    for class_name in class_names:
        class_dir = dataset_path / class_name
        if not class_dir.exists():
            print(f"Warning: {class_dir} not found, skipping...")
            continue
        
        samples = list(class_dir.glob('*.npy'))
        print(f"  {class_name}: {len(samples)} samples")
        
        for sample_path in samples:
            try:
                points = np.load(sample_path)
                all_data.append(points)
                all_labels.append(class_to_idx[class_name])
            except Exception as e:
                print(f"Error loading {sample_path}: {e}")
    
    data = np.array(all_data)
    labels = np.array(all_labels)
    
    print(f"\nTotal samples: {len(data)}")
    print(f"Data shape: {data.shape}")
    
    return data, labels, class_names


def train_epoch(model, dataloader, criterion, optimizer, device):
    """Train for one epoch"""
    model.train()
    running_loss = 0.0
    correct = 0
    total = 0
    
    for batch_idx, (data, target) in enumerate(dataloader):
        data, target = data.to(device), target.to(device)
        
        optimizer.zero_grad()
        output = model(data)
        loss = criterion(output, target)
        loss.backward()
        optimizer.step()
        
        running_loss += loss.item()
        _, predicted = torch.max(output.data, 1)
        total += target.size(0)
        correct += (predicted == target).sum().item()
    
    epoch_loss = running_loss / len(dataloader)
    epoch_acc = 100 * correct / total
    
    return epoch_loss, epoch_acc


def evaluate(model, dataloader, criterion, device):
    """Evaluate model"""
    model.eval()
    running_loss = 0.0
    correct = 0
    total = 0
    all_predictions = []
    all_targets = []
    
    with torch.no_grad():
        for data, target in dataloader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            loss = criterion(output, target)
            
            running_loss += loss.item()
            _, predicted = torch.max(output.data, 1)
            total += target.size(0)
            correct += (predicted == target).sum().item()
            
            all_predictions.extend(predicted.cpu().numpy())
            all_targets.extend(target.cpu().numpy())
    
    epoch_loss = running_loss / len(dataloader)
    epoch_acc = 100 * correct / total
    
    return epoch_loss, epoch_acc, all_predictions, all_targets


def main():
    print("=" * 60)
    print("PointNet Object Classifier Training")
    print("=" * 60)
    
    # Paths
    home = Path.home()
    dataset_path = home / 'multi_modal_perception' / 'data' / 'training_dataset'
    model_dir = home / 'multi_modal_perception' / 'models'
    model_dir.mkdir(exist_ok=True)
    model_path = model_dir / 'object_classifier.pth'
    
    # Device
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"\nUsing device: {device}")
    
    # Load dataset
    data, labels, class_names = load_dataset(dataset_path)
    
    # Split train/test (80/20)
    X_train, X_test, y_train, y_test = train_test_split(
        data, labels, test_size=0.2, random_state=42, stratify=labels
    )
    
    print(f"\nTrain samples: {len(X_train)}")
    print(f"Test samples: {len(X_test)}")
    
    # Create datasets and dataloaders
    train_dataset = PointCloudDataset(X_train, y_train)
    test_dataset = PointCloudDataset(X_test, y_test)
    
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)
    
    # Model
    model = PointNetClassifier(num_classes=len(class_names)).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.5)
    
    # Training
    num_epochs = 50
    best_acc = 0.0
    
    print("\n" + "=" * 60)
    print("Starting Training")
    print("=" * 60)
    
    start_time = time.time()
    
    for epoch in range(num_epochs):
        train_loss, train_acc = train_epoch(model, train_loader, criterion, optimizer, device)
        test_loss, test_acc, _, _ = evaluate(model, test_loader, criterion, device)
        scheduler.step()
        
        print(f"Epoch [{epoch+1:2d}/{num_epochs}] "
              f"Train Loss: {train_loss:.4f} Train Acc: {train_acc:.2f}% | "
              f"Test Loss: {test_loss:.4f} Test Acc: {test_acc:.2f}%")
        
        # Save best model
        if test_acc > best_acc:
            best_acc = test_acc
            torch.save(model.state_dict(), model_path)
            print(f"  ✓ Saved best model (acc: {best_acc:.2f}%)")
    
    training_time = time.time() - start_time
    
    print("\n" + "=" * 60)
    print("Training Complete!")
    print("=" * 60)
    print(f"Total time: {training_time:.1f} seconds")
    print(f"Best test accuracy: {best_acc:.2f}%")
    print(f"Model saved to: {model_path}")
    
    # Final evaluation with best model
    print("\n" + "=" * 60)
    print("Final Evaluation")
    print("=" * 60)
    
    model.load_state_dict(torch.load(model_path))
    _, final_acc, predictions, targets = evaluate(model, test_loader, criterion, device)
    
    print(f"\nFinal Test Accuracy: {final_acc:.2f}%")
    
    # Classification report
    print("\nClassification Report:")
    print(classification_report(targets, predictions, target_names=class_names))
    
    # Confusion matrix
    print("Confusion Matrix:")
    cm = confusion_matrix(targets, predictions)
    print("           ", "  ".join([f"{name:8s}" for name in class_names]))
    for i, row in enumerate(cm):
        print(f"{class_names[i]:8s}", "  ".join([f"{val:8d}" for val in row]))
    
    # Per-class accuracy
    print("\nPer-Class Accuracy:")
    for i, class_name in enumerate(class_names):
        class_correct = cm[i, i]
        class_total = cm[i, :].sum()
        class_acc = 100 * class_correct / class_total if class_total > 0 else 0
        print(f"  {class_name:10s}: {class_acc:.2f}% ({class_correct}/{class_total})")
    
    print("\n" + "=" * 60)
    print("Training script finished successfully!")
    print("You can now run the ML classifier node:")
    print("  python3 src/ml_object_classifier.py")
    print("=" * 60)


if __name__ == '__main__':
    main()