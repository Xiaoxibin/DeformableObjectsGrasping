import os
import torch
import yaml
import numpy as np
from PIL import Image
from torchvision import transforms
from utils import model_factory
from utils.test_data_loader import GraspingSlidingDataset
from torch.utils.data.dataloader import DataLoader

# 检查 CUDA 是否可用
cuda_avail = torch.cuda.is_available()


def test_images(params, model_path, data_path):

    # 图片处理
    transform_rgb = transforms.Compose([
        transforms.Resize((120, 160)),
        transforms.ToTensor()
    ])

    transform_tactile = transforms.Compose([
        transforms.Resize((150, 200)),
        transforms.ToTensor()
    ])


    # 检查是否使用 GPU
    if params['use_gpu'] == 1 and cuda_avail:
        device = torch.device("cuda:0")
        use_gpu = True
    else:
        device = torch.device("cpu")
        use_gpu = False

    # 加载模型
    if params['Modality'] == "Combined":
        NN_model, model_params = model_factory.get_model(params, use_gpu)
    
    # 加载预训练模型权重
    state_dict = torch.load(model_path, map_location=device)
    NN_model.load_state_dict(state_dict['model'])
    
    if use_gpu:
        NN_model = NN_model.cuda()

    NN_model.eval()
    
    # 预处理输入图像
    data_path = '/home/xxb/Downloads/data'
    dataset = GraspingSlidingDataset(data_path, transform_rgb=transform_rgb, transform_tactile=transform_tactile)

    # 创建DataLoader
    dataloader = DataLoader(dataset, batch_size=1, shuffle=False)
    data = next(iter(dataloader))

    # 运行模型推理
    with torch.no_grad():
        if params['Modality'] == "Combined":
            output = NN_model(data[0], data[1], data[2], data[3], torch.tensor([[5.0]])) ## [[5.0]] 为二维数组的原因是因为还有个 batch_size 这一维
        _, predicted = torch.max(output.data, 1)
    
    # 输出预测结果
    print(f"Output: {output}")
    print(f"Predicted: {predicted}")
    return output, predicted

if __name__ == "__main__":
    # 加载配置文件
    with open('/home/xxb/DeformableObjectsGrasping-master/src/grasping_framework/config_cluster.yaml', 'r') as file:
        params = yaml.safe_load(file)

    # 模型路径
    model_path = '/home/xxb/DeformableObjectsGrasping-master/src/grasping_framework/Trained_Model/vivit_fdp_two/03_07_2024__09_54_46/vivit_fdp_two46.pt'

    # 输入图像路径
    data_path = '/home/xxb/Downloads/data'

    # 测试图像
    output, predicted = test_images(params, model_path, data_path)
