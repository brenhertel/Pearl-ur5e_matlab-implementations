import matplotlib.pyplot as plt
import numpy as np

def gradient_map(A, name='', fpath='', min_in=0, max_in=1):
    fig = plt.figure()
    im = plt.imshow(A, cmap='gray', vmin=min_in, vmax=max_in)
    plt.xticks([])
    plt.yticks([])
    plt.colorbar(im)
    plt.title(name, fontsize=32)
    plt.savefig(fpath + name + '.png')
    #plt.show()

def gradient_map_show(A, name='', min_in=0, max_in=1):
    fig = plt.figure()
    im = plt.imshow(A, cmap='gray', vmin=min_in, vmax=max_in)
    plt.xticks([])
    plt.yticks([])
    plt.colorbar(im)
    print(name)
    #plt.title(name, fontsize=32)
    plt.show()

def rgb_gradient(A, B, C, name='', fpath='', min_in=0, max_in=1):
    fig = plt.figure()
    img_stacked = np.dstack((A, B, C))
    im = plt.imshow(img_stacked, vmin=min_in, vmax=max_in)
    plt.xticks([])
    plt.yticks([])
    plt.title(name, fontsize=32)
    plt.savefig(fpath + name + '.png')
    #plt.show()
    
def strongest_gradient(A, B, C, name='', fpath='', min_in=0, max_in=1):
    fig = plt.figure()
    img_stacked = np.dstack((A, B, C))
    for i in range (np.shape(img_stacked)[0]):
        for j in range (np.shape(img_stacked)[1]):
            for k in range (np.shape(img_stacked)[2]):
                if img_stacked[i][j][k] == max(img_stacked[i][j]):
                    img_stacked[i][j] = np.zeros((np.shape(img_stacked)[2]))
                    img_stacked[i][j][k] = 1
    im = plt.imshow(img_stacked, vmin=min_in, vmax=max_in)
    plt.xticks([])
    plt.yticks([])
    plt.title(name, fontsize=32)
    plt.savefig(fpath + name + '.png')
    #plt.show()

#in-file testing
def main():
  A = np.random.random((5, 5))
  B = np.random.random((5, 5))
  C = np.random.random((5, 5))
  print(A)
  print(B)
  print(C)
  gradient_map(A, 'TEST')
  strongest_gradient(A, B, C, name='RGB TEST')

if __name__ == '__main__':
  main()