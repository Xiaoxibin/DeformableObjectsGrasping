import os
import re
import random
import cv2
import torch
from torch.utils import data
from torch import nn

from torch.utils.data.dataloader import DataLoader

# For creating a custom dataset: it needs to contain three funcs: __init__, __len__, __getitem__
# Default: no scale ratio
class Tactile_Vision_dataset(data.Dataset):
    def __init__(self, scale_ratio = 1, video_length = 8, data_path='./data'):
        self.data_path = data_path
        self.label_files = []
        self.train_data = []
        self.scale_percent = scale_ratio
        self.video_length = video_length

        # 遍历数据集下的所有文件和目录
        for root, dirs, files in os.walk(data_path, topdown=True):
            # 遍历当前路径下的文件列表
            for file in files:
                # 判断文件是否以 '.dat' 结尾，这类文件通常包含标签信息
                if file.endswith('.dat'):
                    # 将符合条件的文件路径添加到标签文件列表中
                    self.label_files.append(os.path.join(root, file))
        # 对标签文件路径列表进行排序
        self.label_files.sort()

        # 编译正则表达式模式，用于匹配标签文件名中的对象 ID
        pat = re.compile(r'object([0-9]+)_result')  #filter

        # 遍历标签文件列表中的每个标签文件
        for label_file in self.label_files:

            # 使用正则表达式匹配标签文件名中的对象 ID ,并提取匹配结果
            idx = pat.search(label_file).group(1)
            fp = open(label_file, 'r')

            # 逐行读取标签文件内容，并存储在列表中
            lines = fp.readlines()
            # 将读取到的每行内容末尾添加对象 ID，并将结果添加到训练数据列表中
            self.train_data.extend([line.replace('\n','') + ' ' + idx for line in lines])
        
        ##### 构造函数将标签文件中的信息加载到 'train_data' 列表中，以便后续使用
        

    def __len__(self):
        return len(self.train_data)

    def __getitem__(self, index):  #need to be defined to let data_loader work
        train_data = self.train_data[index]
        output_tactile_imgs = []
        output_rgb_imgs = []
        # 将样本以空格分割为列表
        train_data = train_data.split(' ')
        object_id = train_data[-1]  # object ID
        id_2 = train_data[-3] # Experiment ID (on the same object)

        # status 变量将保存 train_data 列表中索引为 2 的元素的第一个字符所表示的整数值
        status = int(train_data[2][0]) # Label  # 0 -> slip; 1 -> success
        # 将 status 转换成 PyTorch 张量形式
        label = torch.tensor([status]).long()
        # 去除张量中的冗余维度
        label = torch.squeeze(label)
        # 构建图像路径
        path = os.path.join(self.data_path, 'object' + object_id, id_2 + '_mm')
        rgb_img_paths = []
        for root, dirs, files in os.walk(path, topdown=True):
            for file in files:
                if ("external_" in file) and file.endswith('.jpg'):  # select camera images
                    rgb_img_paths.append(os.path.join(root, file))
        rgb_img_paths_selected = []
        # 对 RGB 图像列表进行排序
        rgb_img_paths.sort()  #Sort images
        # 获取起始图像的索引
        start_image = train_data[5]
        # 初始化索引变量
        index = 0

        # 循环直到选择到足够数量的 RGB 图像路劲
        while(len(rgb_img_paths_selected) < self.video_length):  # 8 frames per time (LSTM)
            if start_image in rgb_img_paths[index] or len(rgb_img_paths_selected) > 0:
                rgb_img_paths_selected.append(rgb_img_paths[index])
            index += 1
        index = 0

        # 构造对应的触觉图像路径 cor_tactile_img_path，通过将 rgb_img_path 中的字符串 'external' 替换为 'gelsight'。
        # 使用 OpenCV 的 cv2.imread() 函数读取 rgb_img_path 和 cor_tactile_img_path 对应的图像，并分别保存到 rgb_img 和 tactile_img 变量中
        for rgb_img_path in rgb_img_paths_selected:
            cor_tactile_img_path = rgb_img_path.replace('external', 'gelsight')
            rgb_img = cv2.imread(rgb_img_path)
            tactile_img = cv2.imread(cor_tactile_img_path)
            size = rgb_img.shape  # 480, 640, 3 (width, height, channel)
            # new width / new height = 480 / 640 * scale_percent

            rgb_img_resized = cv2.resize(rgb_img, (int(size[1] * self.scale_percent), int(size[0] * self.scale_percent)), interpolation = cv2.INTER_AREA)
            # rgb_img_resized = cv2.resize(rgb_img,(224, 224),interpolation=cv2.INTER_AREA)
            tactile_img_resized = cv2.resize(tactile_img, (int(size[1] * self.scale_percent), int(size[0] * self.scale_percent)), interpolation=cv2.INTER_AREA)
            size = rgb_img_resized.shape
            # size = tactile_img_resized.shape
            # 将图像转换成 PyTorch 张量，并转置通道
            rgb_img_tensor = torch.from_numpy(rgb_img_resized.transpose(2,0,1)).float()

            #turn into a tensor (3, 240, 320)  -> resized one
            tactile_img_tensor = torch.from_numpy(tactile_img_resized.transpose(2,0,1)).float()

            # 如果 index 为0，表示第一次读取图像，将三维图像扩展维度变成四维，以便后续拼接
            # 反之，则不是第一次读取图像，则使用 torch.cat() 函数将当前张量添加到之前存储的张量中
            if index == 0:
                output_rgb_imgs = rgb_img_tensor[None,:]
                output_tactile_imgs = tactile_img_tensor[None,:]
            else:
                output_rgb_imgs = torch.cat([output_rgb_imgs, rgb_img_tensor[None,:]], dim=0)
                output_tactile_imgs = torch.cat([output_tactile_imgs, tactile_img_tensor[None,:]], dim=0)
            index += 1
        return output_rgb_imgs.transpose(0, 1), output_tactile_imgs.transpose(0, 1), label # rgb images; visual images; label

if __name__ == "__main__":
    # set a global dataset path
    train_dataset = Tactile_Vision_dataset(data_path = '/home/xxb/Downloads/Slip_small_dataset/Small Dataset/training')
    train_data_loader = DataLoader(train_dataset, batch_size=4, shuffle=True,
                                   num_workers=4)
    data = next(iter(train_data_loader))
    #用于获取下一批次的数据
    print("Print feed-forward test results:")
    print(data[0].shape)
    print(data[1].shape)
    print(data[2].shape)

    # 使用len函数获取信息并输出
    length = len(train_dataset)
    print("Length of train data:", length)

    # print(train_dataset[0][0])
    # for i in range(10):
    #     output_rgb_imgs, output_tactile_imgs, label = train_dataset[i]
    #     print(output_rgb_imgs[0].shape)
    #     print(output_tactile_imgs[0].shape)
    #     print(label)

    # for i in range(1000):
    #     train_dataset[i]
    #     print(i)
