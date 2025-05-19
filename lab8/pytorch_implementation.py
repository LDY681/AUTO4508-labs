
import torch
from torch import nn
from functools import partial

from torchvision import datasets, models, transforms
from PIL import Image

from einops import rearrange, repeat
from einops.layers.torch import Rearrange
from model import VSSM, SS2D

from torch.utils.tensorboard import SummaryWriter

import numpy as np
from torchsummary import summary

import cv2
import os
import sys
from os.path import join, exists, dirname, abspath

from sklearn.model_selection import train_test_split

import torch.optim as optim
from torch.optim.lr_scheduler import StepLR

writer = SummaryWriter()



class your_model(nn.Module):
    def __init__(
            self,
            patch_size=4, 
            in_chans=3, 
            num_classes=1000, 
            imgsize=224,
            **kwargs,
    ):
        super().__init__()


        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.base_model = nn.Conv2d(3, 24, kernel=(5,5), stride=(2,2))
        self.base_activation = nn.ELU()

        self.mlp_head1 = nn.Sequential(
                nn.Linear(num_classes, int(num_classes / 2)),
                nn.BatchNorm1d(int(num_classes / 2)),
                nn.ELU(),

                
                nn.Linear(int(num_classes / 4), 1)).to(device=self.device)
        

    def forward(self, img):
        
        x = self.base_model(img)
        x = self.base_activation(x)
        b, n = x.shape
        speed = self.mlp_head1(x)
        angle = self.mlp_head2(x)
        print(x.shape)
        return speed[:,0], angle[:,0]

transform = transforms.Compose(
    [
        transforms.ToTensor(),
    ]
)

class dataset(torch.utils.data.Dataset):
    def __init__(self, file_list, label_list1, label_list2, transform=None):
        self.file_list = file_list
        self.label_list1 = label_list1
        self.label_list2 = label_list2
        self.transform = transform

    def __len__(self):
        self.filelength = len(self.file_list)
        return self.filelength

    def __getitem__(self, idx):
        img = self.file_list[idx]
        label1 = self.label_list1[idx]
        label2 = self.label_list2[idx]
        img_transformed = self.transform(img)
        return img_transformed, label1, label2

if __name__ == "__main__":
    lr = 1e-3
    gamma = 0.4
    batch_size = 20

    model = your_model(
        self,
        patch_size=4, 
        in_chans=3, 
        num_classes=1000, 
        imgsize=(200, 320),
        **kwargs,
    )


    summary(model, (3, 200, 320))

    #retrieve all the data to train the model

    Images_All_Straight = []
    Speeds_All_Straight = []
    Steering_Angles_All_Straight = []

    Images_All_Turn = []
    Speeds_All_Turn = []
    Steering_Angles_All_Turn = []

    Images_All = []
    Speeds_All = []
    Steering_Angles_All = []

    Script_Path = abspath(join(join(join(join((__file__), '..'), '..'), '..'), 'eyesim'))
    Image_Path = join(Script_Path, 'Image Datasets')
    print(Image_Path)
    if exists(Image_Path):
        for Folders in os.listdir(Image_Path):
            Search_Folder = join(Image_Path, Folders)
            for Files in os.listdir(Search_Folder):
                Speed=int(Files.split("_")[3])
                Steering_Angle=int(Files.split("_")[4])
                img=Image.open(join(Search_Folder, Files)).convert("RGB")
                if Steering_Angle < 1:
                    Images_All_Straight.append(img)
                    Speeds_All_Straight.append(Speed)
                    Steering_Angles_All_Straight.append(Steering_Angle)
                else:
                    Images_All_Turn.append(img)
                    Speeds_All_Turn.append(Speed)
                    Steering_Angles_All_Turn.append(Steering_Angle)
    else:
        print('[Error!] Check Image Directory is Correct!')
        sys.exit()   

    print(len(Images_All_Turn))
    print(len(Images_All_Straight))
    print(int(len(Images_All_Straight)/len(Images_All_Turn)))

    for i in range(int(len(Images_All_Straight)/len(Images_All_Turn))):
        Images_All = Images_All + Images_All_Turn
        Speeds_All = Speeds_All + Speeds_All_Turn
        Steering_Angles_All = Steering_Angles_All + Steering_Angles_All_Turn

    Images_All = Images_All + Images_All_Straight
    Speeds_All = Speeds_All + Speeds_All_Straight
    Steering_Angles_All = Steering_Angles_All + Steering_Angles_All_Straight

    # Split the data into training, test and validation sets

    Split_a = train_test_split(Images_All, Speeds_All, Steering_Angles_All, test_size=0.1, shuffle=True)
    (Images, Image_Test, Speeds, Speed_Test, Steering_Angles, Steering_Angle_Test) = Split_a   
    Split_b = train_test_split(Images, Speeds, Steering_Angles, test_size=0.2, shuffle=True)
    (Image_Train, Image_Valid, Speed_Train, Speed_Valid, Steering_Angle_Train, Steering_Angle_Valid) = Split_b

    train_data = dataset(Image_Train, Speed_Train, Steering_Angle_Train, transform=transform)
    test_data = dataset(Image_Test, Speed_Test, Steering_Angle_Test, transform=transform)
    val_data = dataset(Image_Valid, Speed_Valid, Steering_Angle_Valid, transform=transform)

    train_loader = torch.utils.data.DataLoader(
        dataset=train_data, batch_size=batch_size, shuffle=False
    )
    test_loader = torch.utils.data.DataLoader(
        dataset=test_data, batch_size=batch_size, shuffle=True
    )
    val_loader = torch.utils.data.DataLoader(
        dataset=val_data, batch_size=batch_size, shuffle=True
    )


    # setup the training conditions

    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = StepLR(optimizer, step_size=10, gamma=gamma)

    # start training
    epochs = 25
    device = "cuda" if torch.cuda.is_available() else "cpu"

    for epoch in range(epochs):
        epoch_loss = 0
        epoch_loss1 = 0
        epoch_loss2 = 0
        epoch_accuracy1 = 0
        epoch_accuracy2 = 0

        for data, label1, label2 in train_loader:
            data = data.to(device)
            label1 = label1.float().to(device)
            label2 = label2.float().to(device)

            output1, output2 = model(data)
            loss1 = criterion(output1, label1)
            loss2 = criterion(output2, label2)

            loss = loss1 + loss2

            writer.add_scalar("Loss/Train", loss, epoch)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            acc1 = (abs(output1 - label1) < (15 / 300)).float().mean()
            acc2 = (abs(output2 - label2) < (8 / 60)).float().mean()
            epoch_accuracy1 += acc1 / len(train_loader)
            epoch_accuracy2 += acc2 / len(train_loader)
            epoch_loss += loss / len(train_loader)
            epoch_loss1 += loss1 / len(train_loader)
            epoch_loss2 += loss2 / len(train_loader)

        print(
            "Epoch : {}, train accuracy1 : {}, train accuracy2 : {}, train loss : {}".format(
                epoch + 1, epoch_accuracy1, epoch_accuracy2, epoch_loss
            )
        )

        print(
            "Individual Epoch : {}, train loss1 : {}, train loss2 : {}".format(
                epoch + 1, epoch_loss1, epoch_loss2
            )
        )

        with torch.no_grad():
            epoch_val_accuracy1 = 0
            epoch_val_accuracy2 = 0
            epoch_val_loss = 0
            for data, label1, label2 in val_loader:
                data = data.to(device)
                label1 = label1.float().to(device)
                label2 = label2.float().to(device)

                val_output1, val_output2 = model(data)
                val_loss1 = criterion(val_output1, label1)
                val_loss2 = criterion(val_output2, label2)

                val_loss = val_loss1 + val_loss2

                acc1 = (abs(val_output1 - label1) < (15 / 300)).float().mean()
                acc2 = (abs(val_output2 - label2) < (8 / 60)).float().mean()
                epoch_val_accuracy1 += acc1 / len(val_loader)
                epoch_val_accuracy2 += acc2 / len(val_loader)
                epoch_val_loss += val_loss / len(val_loader)

            print(
                "Epoch : {}, val_accuracy : {}, val_accuracy : {}, val_loss : {}".format(
                    epoch + 1, epoch_val_accuracy1, epoch_val_accuracy2, epoch_val_loss
                )
            )

        scheduler.step()
    
    torch.save(model, 'model_name.pth')
    writer.flush()
    writer.close()

