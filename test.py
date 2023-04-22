import time
import blocks
import surface
import matplotlib.pyplot as plt



# start = time.time()
# # Example usage
# #photo = [[random.randint(0,1) for j in range(5000)] for i in range(5000)]
# photo = [
#    [1, 0, 0, 1, 1, 1, 0],
#     [0, 1, 0, 0, 0, 0, 1],
#     [0, 1, 0, 0, 0, 0, 0],
#     [1, 1, 0, 1, 1, 1, 1],
# ]
# #photo = condition
# clas = blocks.block.classifyRouteBlocks(photo)
# print(clas)
# end = time.time()
# print(end - start)
# print("size of spots: " +str(len(clas)))
# print("ok")

t = surface.surface('dsm_1m_utm18_e_10_104.tif','dtm_1m_utm18_e_10_104.tif',0.5,50)
diff = t.getDiffs()
print(diff[5,7])
cond = (diff<=50).astype(int)
cond *= 255

plt.imshow(cond, cmap='gray')
plt.show()