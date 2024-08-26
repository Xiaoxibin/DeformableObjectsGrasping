import torch
import torch.nn as nn
import math
import warnings
import torch.nn.functional as F
import numpy as np

from torch import Tensor
from einops import rearrange, reduce, repeat

def drop_path(x, drop_prob: float = 0., training: bool = False):
    """Drop paths (Stochastic Depth) per sample (when applied in main path of residual blocks).
    This is the same as the DropConnect impl I created for EfficientNet, etc networks, however,
    the original name is misleading as 'Drop Connect' is a different form of dropout in a separate paper...
    See discussion: https://github.com/tensorflow/tpu/issues/494#issuecomment-532968956 ... I've opted for
    changing the layer and argument names to 'drop path' rather than mix DropConnect as a layer name and use
    'survival rate' as the argument.
    """
    if drop_prob == 0. or not training:
        return x
    keep_prob = 1 - drop_prob
    shape = (x.shape[0],) + (1,) * (x.ndim - 1)  # work with diff dim tensors, not just 2D ConvNets
    random_tensor = keep_prob + torch.rand(shape, dtype=x.dtype, device=x.device)
    random_tensor.floor_()  # binarize
    output = x.div(keep_prob) * random_tensor
    return output

class DropPath(nn.Module):
    """Drop paths (Stochastic Depth) per sample  (when applied in main path of residual blocks).
    """
    def __init__(self, drop_prob=None):
        super(DropPath, self).__init__()
        self.drop_prob = drop_prob

    def forward(self, x):
        return drop_path(x, self.drop_prob, self.training)

def _no_grad_trunc_normal_(tensor, mean, std, a, b):
    def norm_cdf(x):
        # Computes standard normal cumulative distribution function
        return (1. + math.erf(x / math.sqrt(2.))) / 2.

    if (mean < a - 2 * std) or (mean > b + 2 * std):
        warnings.warn("mean is more than 2 std from [a, b] in nn.init.trunc_normal_. "
                      "The distribution of values may be incorrect.",
                      stacklevel=2)

    with torch.no_grad():
        # Values are generated by using a truncated uniform distribution and
        # then using the inverse CDF for the normal distribution.
        # Get upper and lower cdf values
        l = norm_cdf((a - mean) / std)
        u = norm_cdf((b - mean) / std)

        # Uniformly fill tensor with values from [l, u], then translate to
        # [2l-1, 2u-1].
        tensor.uniform_(2 * l - 1, 2 * u - 1)

        # Use inverse cdf transform for normal distribution to get truncated
        # standard normal
        tensor.erfinv_()

        # Transform to proper mean, std
        tensor.mul_(std * math.sqrt(2.))
        tensor.add_(mean)

        # Clamp to ensure it's in the proper range
        tensor.clamp_(min=a, max=b)
        return tensor

def trunc_normal_(tensor, mean=0., std=1., a=-2., b=2.):
    # type: (Tensor, float, float, float, float) -> Tensor
    r"""Fills the input Tensor with values drawn from a truncated
    normal distribution. The values are effectively drawn from the
    normal distribution :math:`\mathcal{N}(\text{mean}, \text{std}^2)`
    with values outside :math:`[a, b]` redrawn until they are within
    the bounds. The method used for generating the random values works
    best when :math:`a \leq \text{mean} \leq b`.
    Args:
        tensor: an n-dimensional `torch.Tensor`
        mean: the mean of the normal distribution
        std: the standard deviation of the normal distribution
        a: the minimum cutoff value
        b: the maximum cutoff value
    Examples:
        >>> w = torch.empty(3, 5)
        >>> nn.init.trunc_normal_(w)
    """
    return _no_grad_trunc_normal_(tensor, mean, std, a, b)


class Mlp(nn.Module):
    def __init__(self, in_features, hidden_features=None, out_features=None,  drop=0.):
        super().__init__()
        out_features = out_features or in_features
        hidden_features = hidden_features or in_features
        self.fc1 = nn.Linear(in_features, hidden_features)
        self.act = nn.GELU()
        self.fc2 = nn.Linear(hidden_features, out_features)
        self.drop = nn.Dropout(drop)

    def forward(self, x):
        x = self.fc1(x)
        x = self.act(x)
        x = self.drop(x)
        x = self.fc2(x)
        x = self.drop(x)
        return x


class FDPAttention(nn.Module):
    def __init__(self,dim,num_heads, qkv_bias,attn_drop, proj_drop):
        super().__init__()
        self.num_heads = num_heads
        head_dim = dim // num_heads
        self.scale = head_dim ** -0.5
        self.qkv_s = nn.Linear(dim, 3*dim, bias=qkv_bias)
        self.qkv_t = nn.Linear(dim, 3*dim, bias=qkv_bias)
        self.attn_drop_s = nn.Dropout(attn_drop)
        self.attn_drop_t = nn.Dropout(attn_drop)
        self.proj = nn.Linear(2*dim,dim)
        self.proj_drop = nn.Dropout(proj_drop)

    def forward(self,x):
        B,T,HW,D = x.shape

        #Spatial attention
        qkv_s = self.qkv_s(x) #Batch_size, frames(time), height*width, embed_dim*3
        qkv_s = qkv_s.reshape(B*T,HW,3,self.num_heads//2, (D*2) // self.num_heads) #Batch_size*frames(time), height*width, 3, num_heads//2, D*2//self.num_heads 

        qkv_s = qkv_s.permute(2,0,3,1,4) #3, Batch_size*frames(time), num_heads//2, HW, 2*D//num_heads
        qs, ks, vs = qkv_s[0], qkv_s[1], qkv_s[2] #Batch_size*frames(time), num_heads//2, HW, 2*D//num_heads
        attn_s = (qs @ ks.transpose(-2,-1)) * self.scale #Batch_size*frames(time), num_heads//2, HW, HW
        attn_s = attn_s.softmax(dim=-1)
        attn_s = self.attn_drop_s(attn_s)
        x_s = (attn_s @ vs).transpose(1,2) #Batch_size*frames(time), num_heads//2, HW, 2*D//num_heads
        x_s = x_s.transpose(1,2) #Batch_size*frames(time), HW,num_heads//2, 2*D//num_heads
        x_s = x_s.reshape(B,T,HW,D) #Batch_size, frames(time), HW, D

        #Temporal attention
        qkv_t = self.qkv_t(x) #Batch_size, frames(time), height*width, embed_dim*3
        qkv_t = qkv_t.transpose(1,2) #Batch_size,  height*width, frames(time),embed_dim*3
        qkv_t = qkv_t.reshape(B*HW,T,3,self.num_heads//2, (D*2) // self.num_heads) #Batch_size* height*width, frames(time), 3, num_heads//2, D*2//self.num_heads 

        qkv_t = qkv_t.permute(2,0,3,1,4) #3, Batch_size*HW, num_heads//2, frames(time), 2*D//num_heads
        qt, kt, vt = qkv_t[0], qkv_t[1], qkv_t[2] #Batch_size*HW, num_heads//2, frames(time),  2*D//num_heads
        attn_t = (qt @ kt.transpose(-2,-1)) * self.scale #Batch_size*HW,  num_heads//2, frames(time), frames(time),
        attn_t = attn_t.softmax(dim=-1)
        attn_t = self.attn_drop_t(attn_t)
        x_t = (attn_t @ vt).transpose(1,2) #Batch_size*HW,  num_heads//2, frames(time), 2*D//num_heads
        x_t = x_t.transpose(1,2) #Batch_size*HW,frames(time),num_heads//2, 2*D//num_heads
        x_t = x_t.reshape(B,HW,T,D) #Batch_size, HW, frames(time), D
        x_t = x_t.transpose(1,2)#Batch_size, frames(time),  HW,D

        #print(x_s.shape)
        #print(x_t.shape)
        x = torch.cat((x_s,x_t),dim=3)
        #print(x.shape)
        x = self.proj(x)
        x = self.proj_drop(x)

        return x

class FDTBlock(nn.Module):

    def __init__(self, dim, num_heads, mlp_ratio=4., qkv_bias=False, qk_scale=None, drop=0., attn_drop=0.,
                 drop_path=0.1):
        super().__init__()

        self.norm1 = nn.LayerNorm(dim) #Layer Norm1
        self.attn = FDPAttention(dim, num_heads=num_heads, qkv_bias=qkv_bias,  attn_drop=attn_drop, proj_drop=drop) #Factorized Dot-Product Attention

        ## drop path
        self.drop_path = DropPath(drop_path) if drop_path > 0. else nn.Identity()
        self.norm2 = nn.LayerNorm(dim) #Layer Norm2
        mlp_hidden_dim = int(dim * mlp_ratio) 
        self.mlp = Mlp(in_features=dim, hidden_features=mlp_hidden_dim)

    def forward(self, x, B, T, W):
            #print(f'FDT: {x.shape}')
            H = x.size(1) // W
            x = rearrange(x, '(b t) (h w) d -> b t (h w) d', b=B,h=H,w=W,t=T)
            x = x + self.drop_path(self.attn(self.norm1(x)))
            x = x + self.drop_path(self.mlp(self.norm2(x)))
            x = rearrange(x, 'b t (h w) d -> (b t) (h w) d', b=B,h=H,w=W,t=T)
            return x


class UniformPatchEmbed(nn.Module):
    def __init__(self, img_size, patch_size, in_chans, embed_dim):
        super().__init__()
        num_patches = (img_size[1] // patch_size[1]) * (img_size[0] // patch_size[0])
        self.img_size = img_size
        self.patch_size = patch_size
        self.num_patches = num_patches
    
        #print(f'inchans:{in_chans} embed_dim:{embed_dim} kernel_size:{patch_size} stride:{patch_size}')
        self.proj = nn.Conv2d(in_chans, embed_dim, kernel_size=patch_size, stride=patch_size)

    def forward(self, x):
        B, C, T, H, W = x.shape #Batch, channels, frames(time), height, width
        x = rearrange(x, 'b c t h w -> (b t) c h w') #Combine batch and frames 
        x = self.proj(x) #Create patches using convolution. Trick from the timeSFormer paper. Batch, embed_dim, num_patch_across_height, num_patch_across_width
        W = x.size(-1) #num_patches across_width
        #x = x.flatten(2).transpose(1, 2)  #Flatten patches into 1-dim and make embed_dim the last
        x = x.flatten(2).transpose(1,2) #Batch,num_patches,D
        return x, T, W



class VIVIT(nn.Module):
    def __init__(self,img_size, patch_size, in_chans, num_classes, embed_dim, depth, num_heads, mlp_ratio,qkv_bias,qk_scale, drop_rate, attn_drop_rate, drop_path_rate,num_frames,dropout, use_gpu=False):

        super().__init__()
        self.use_gpu = use_gpu
        self.depth = depth
        self.num_heads = num_heads
        self.dropout = nn.Dropout(dropout)
        self.num_classes = num_classes


        #Uniform sampling
        #simply split each frame into patches of size patch_size and then concatenate for all frames to
        #form the batch
        self.patch_embed_x = UniformPatchEmbed(img_size=img_size,patch_size=patch_size,in_chans=in_chans,embed_dim=embed_dim)
        num_patches_x = self.patch_embed_x.num_patches
        self.patch_embed_y = UniformPatchEmbed(img_size=img_size,patch_size=patch_size,in_chans=in_chans,embed_dim=embed_dim)
        num_patches_y = self.patch_embed_y.num_patches
        if self.use_gpu == True:
            self.patch_embed_x.to('cuda')

        #Cls token
        #self.cls_token = nn.Parameter(torch.zeros(1,1,embed_dim))
        #self.pos_embed = nn.Parameter(torch.zeros(1,num_patches+1, embed_dim))
        self.pos_embed_x = nn.Parameter(torch.zeros(1,num_patches_x, embed_dim))
        self.pos_embed_y = nn.Parameter(torch.zeros(1, num_patches_y, embed_dim))
        self.pos_drop = nn.Dropout(p=drop_rate)

        #Pos embedding

        #Attention blocks
        dpr = [x.item() for x in torch.linspace(0, drop_path_rate, self.depth)]  # stochastic depth decay rule
        self.blocks_x = nn.ModuleList([
            FDTBlock(
                dim=embed_dim, num_heads=num_heads, mlp_ratio=mlp_ratio, qkv_bias=qkv_bias, qk_scale=qk_scale,
                drop=drop_rate, attn_drop=attn_drop_rate, drop_path=dpr[i])
            for i in range(self.depth)])

        self.blocks_y = nn.ModuleList([
            FDTBlock(
                dim=embed_dim, num_heads=num_heads, mlp_ratio=mlp_ratio, qkv_bias=qkv_bias, qk_scale=qk_scale,
                drop=drop_rate, attn_drop=attn_drop_rate, drop_path=dpr[i])
            for i in range(self.depth)])
        self.norm = nn.LayerNorm(embed_dim)

        #Classifier head
        self.head = nn.Linear(2*embed_dim,num_classes) if num_classes > 0 else nn.Identity()

        trunc_normal_(self.pos_embed_x, std=.02)
        trunc_normal_(self.pos_embed_y, std=.02)
        #trunc_normal_(self.cls_token, std=.02)
        #self.apply(self._init_weights)

    def init_weights(self):
        def _init_weights(m):
            if isinstance(m, nn.Linear):
                trunc_normal_(m.weight, std=.02)
                if isinstance(m, nn.Linear) and m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.LayerNorm):
                nn.init.constant_(m.bias, 0)
                nn.init.constant_(m.weight, 1.0)
        self.apply(_init_weights)

    def forward_features_x(self,x):
        B = x.shape[0]
        x, T, W = self.patch_embed_x(x)
        H = x.size(1) // W
        #cls_tokens = self.cls_token.expand(x.size(0), -1, -1)
        #x = torch.cat((cls_tokens, x), dim=1)

        x = x + self.pos_embed_x
        x = self.pos_drop(x)

        ## Attention blocks
        for blk in self.blocks_x:
            x = blk(x, B, T, W)

        #print(f'VIVIT_FF: {x.shape}')
        x = rearrange(x, '(b t) (h w) d -> b (t h w) d', b=B,h=H,w=W,t=T)
        #print(f'VIVIT_FF: {x.shape}')
        x = self.norm(x)
        return x

    def forward_features_y(self,x):
        B = x.shape[0]
        x, T, W = self.patch_embed_y(x)
        H = x.size(1) // W
        #cls_tokens = self.cls_token.expand(x.size(0), -1, -1)
        #x = torch.cat((cls_tokens, x), dim=1)

        x = x + self.pos_embed_y
        x = self.pos_drop(x)

        ## Attention blocks
        for blk in self.blocks_y:
            x = blk(x, B, T, W)

        #print(f'VIVIT_FF: {x.shape}')
        x = rearrange(x, '(b t) (h w) d -> b (t h w) d', b=B,h=H,w=W,t=T)
        #print(f'VIVIT_FF: {x.shape}')
        x = self.norm(x)
        return x

    def forward(self,x,y):
        if self.use_gpu == True:
            x = x.to('cuda')
            y = y.to('cuda')

        x = self.forward_features_x(x)
        y = self.forward_features_y(y)
        #print(x.shape)
        #print(y.shape)
        x = torch.mean(x, dim = 1).view(x.shape[0], -1)
        y = torch.mean(y, dim = 1).view(y.shape[0], -1)
        x = torch.cat((x,y),dim=-1)
        x = self.head(x)

        #print(x.shape)
        return x

if __name__ == '__main__':
    '''
    torch.manual_seed(4)
    x = torch.zeros(2, 3, 2, 4, 4)

    for b in range(x.shape[0]):
        for c in range(x.shape[1]):
            for t in range(x.shape[2]):
                for h in range(x.shape[3]):
                    for w in range(x.shape[4]):
                        x[b,c,t,h,w] =  b*x.shape[1]*x.shape[2]*x.shape[3]*x.shape[4] + t*x.shape[3]*x.shape[4] + h*x.shape[4] + w

    print(x)

    pe = UniformPatchEmbed(img_size=(4,4),patch_size=(2,2),in_chans=3,embed_dim=4)
    out = pe(x)
    print(out[0].shape)
    print(out[0])
    '''
    
    use_gpu = True 
    cuda_avail = torch.cuda.is_available()
    x = torch.rand(4, 3, 8, 120, 160)
    y = torch.rand(4, 3, 8, 120, 160)
    if(use_gpu and cuda_avail):
        use_gpu = True
        x = x.to('cuda')
        y = y.to('cuda')
    else:
        use_gpu = False

    vivit = VIVIT(img_size=(120,160), patch_size=(12,16), in_chans=3, num_classes=10, embed_dim=128, depth=1, num_heads=4, mlp_ratio=4, qkv_bias=False, qk_scale=None, drop_rate=0, attn_drop_rate=0., drop_path_rate=0.,  num_frames=8, dropout=0., use_gpu=use_gpu)
    if(use_gpu):
        vivit = vivit.to('cuda')
    out = vivit(x,y)
    print(out)
