import argparse
from torchvision.utils import save_image
from torch.autograd import Variable
import torch.nn as nn
import torch.nn.functional as F
import torch
import numpy as np
from tqdm import tqdm
from info_data import GetBatchLoaderData
from info_model.model1.model import Generator,weights_init_normal
model_name = "model1"

def adjust_learning_rate(optimizer, cur_iter, max_iters, lr_pow, set_lr, warmup_steps):

    warm_lr = 0.000002
    if cur_iter < warmup_steps:
        linear_step = set_lr - warm_lr
        lr = warm_lr + linear_step * (cur_iter / warmup_steps)
    else:
        scale_running_lr = ((1. - float(cur_iter) / max_iters) ** lr_pow)
        lr = set_lr * scale_running_lr

    for param_group in optimizer.param_groups:
        param_group['lr'] = lr

    return lr

def get_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n_epochs", type=int, default=1000, help="number of epochs of training")
    parser.add_argument("--batch_size", type=int, default=16, help="size of the batches")
    parser.add_argument("--lr", type=float, default=0.00002, help="adam: learning rate")
    parser.add_argument("--b1", type=float, default=0.5, help="adam: decay of first order momentum of gradient")
    parser.add_argument("--b2", type=float, default=0.999, help="adam: decay of first order momentum of gradient")
    parser.add_argument("--n_cpu", type=int, default=8, help="number of cpu threads to use during batch generation")
    parser.add_argument("--img_size",  type=int, default=100, help="size of each image dimension")
    parser.add_argument("--in_channels", type=int, default=1, help="number of image channels")
    parser.add_argument("--out_channels", type=int, default=1, help="number of image channels")
    parser.add_argument("--sample_interval", type=int, default=400, help="interval between image sampling")
    opt = parser.parse_args()
    print(opt)
    return opt

def test(model,data,loss_func,device):
    model.eval()
    loss = []
    for j,(x,y) in enumerate(tqdm(data)):
        inform = Variable(y.type(device))
        map = Variable(x.type(device))
        gen_imgs = model(map)
        g_loss = loss_func(gen_imgs,inform)
        loss.append(g_loss.item())
    return np.mean(loss)


opt = get_parse()
cuda = True if torch.cuda.is_available() else False
# cuda = False

# Loss function
seg_loss = torch.nn.MSELoss(reduction='mean')

# Initialize generator and discriminator
generator = Generator(opt.in_channels, opt.out_channels)

if cuda:
    generator.cuda()
    seg_loss.cuda()

# Initialize weights
generator.apply(weights_init_normal)

# Configure data loader
dataloader = GetBatchLoaderData("E:\\python\\GAN\\info_map_data",opt.batch_size)
test_dataloader = GetBatchLoaderData("E:\\python\\GAN\\info_map_data",opt.batch_size,"test")

# Optimizers
optimizer_G = torch.optim.Adam(generator.parameters(), lr=opt.lr, betas=(opt.b1, opt.b2))

Tensor = torch.cuda.FloatTensor if cuda else torch.FloatTensor

# ----------
#  Training
# ----------
datasize = len(dataloader)
max_iters = datasize*opt.n_epochs
totals = 0
best_loss = 0.1
for epoch in range(opt.n_epochs):
    train_loss = []
    test_loss = []
    for i, (map, inform) in enumerate(tqdm(dataloader)):
        totals += 1

        # Configure input
        inform = Variable(inform.type(Tensor))
        map = Variable(map.type(Tensor))

        # Set lr
        now_lr = adjust_learning_rate(optimizer_G, totals, max_iters, lr_pow=1, set_lr=opt.lr,
                                           warmup_steps=2000)

        optimizer_G.zero_grad()
        gen_imgs = generator(map)
        g_loss = seg_loss(gen_imgs,inform)
        train_loss.append(g_loss.item())
        g_loss.backward()
        optimizer_G.step()

    print("testing")
    test_loss.append(test(generator,test_dataloader,seg_loss,Tensor))
    print( "[Epoch %d/%d]  [train loss: %f]  [lr: %f]  [test loss: %f]"
            % (epoch+1, opt.n_epochs, np.mean(train_loss), now_lr, test_loss[-1]))

    np.save("info_model/"+model_name+"/train_loss.npy",train_loss)
    np.save("info_model/"+model_name+"/test_loss.npy",test_loss)
    if np.mean(train_loss)<best_loss:
        best_loss = np.mean(train_loss)
        torch.save(generator.state_dict(),"info_model/"+model_name+"/best_model_params.pth")
        print("saved best model, best loss is ", best_loss)
