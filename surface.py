import rasterio
import numpy as np
import blocks as blocks
import random
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

Image.MAX_IMAGE_PIXELS = 100000000
dsm_blocks_num = 2500
dsm_block_size = 100

class surface:

    def __init__(self, dtm_path,sold_th,drone_th):
        # blocks - cluster of lists of valid locations for soldiers
        # drone_map - matrix of valid locations for drones where 1 = valid
        self.dsm, self.dsm_blocks, self.dsm_city_grid, self.dsm_img, self.dtm, self.diffs_dsm_dtm, \
            self.blocks, self.white_block_indexes, self.drone_map = loadSurface(dtm_path,sold_th,drone_th)
        self.charging_points = [(1000*i,1000*j) for i in range(1,5) for j in range(1,5)]
    
    def getDSM(self):
        return self.dsm

    def getDTM(self):
        return self.dtm

    def getDiffs(self):
        return self.diffs_dsm_dtm

    def getBlocks(self):
        return self.blocks
    
    def getDroneMap(self):
        return self.drone_map
    
    def getWhiteIndexes(self):
        return self.white_block_indexes
    
    def getDSMBlocks(self):
        return self.dsm_blocks
    
    def getDSMLines(self):
        return self.dsm_city_grid
    
    def getDSMImage(self):
        return self.dsm_img
    
    def getChargePoints(self):
        return self.charging_points

def loadSurface(dtm_path,sold_th, drone_th):
    dsm_addition, dsm_blocks, dsm_city_grid, dsm_img = loadGeneratedDSM()
    dtm = np.load(dtm_path)
    dtm = dtm[0][5000:,5000:]
    dtm_max_for_mask = getDTMMask(dtm)
    dtm[dsm_addition!=0] = 0
    dtm_max_for_mask[dsm_addition==0] = 0
    dtm += dtm_max_for_mask
    dsm = dsm_addition
    dsm = dtm+dsm_addition
    diffs = dsm-dtm
    indices = np.where(dsm_addition==0)
    # a list of list because there should be backwards compatibility
    white_block_indexes = list(zip(indices[0], indices[1])) #NPV: there is only one block in the new DSM implementation
    white_block = (dsm_addition==0).astype(int) #1 if a valid point to be for a soldier
    drone_map = (diffs<=drone_th).astype(int)
    drone_map_zeros = np.argwhere(drone_map==0)
    for ind in drone_map_zeros:
        for x_add in range(-10,11):
            for y_add in range(-10,11):
                if (0<=ind[0]+x_add<5000 and 0<=ind[1]+y_add<5000) and (drone_map[ind[0]+x_add,ind[1]+y_add]==0) \
                    and ((x_add)**2 + (y_add)**2 <= 10**2):
                    drone_map[ind[0]+x_add,ind[1]+y_add] = 0
    return dsm, dsm_blocks, dsm_city_grid, dsm_img, dtm, diffs, white_block,\
          white_block_indexes, drone_map

def getDTMMask(dtm):
    dtm_mask = np.copy(dtm)
    for block_num in range(dsm_blocks_num):
        block_init_x = 100*(block_num%50)
        block_init_y = 100*(block_num//50)
        block_dtm = dtm[block_init_x:block_init_x+dsm_block_size-1,block_init_y:block_init_y+dsm_block_size-1]
        max_dtm_val = np.max(block_dtm)
        dtm_mask[block_init_x:block_init_x+dsm_block_size-1,block_init_y:block_init_y+dsm_block_size-1] = max_dtm_val
    return dtm_mask

def findSoldiersValidLocations(diffs, threshold):
    condition = (diffs<=threshold).astype(int)
    validLoc = np.where(condition==1)
    return validLoc #returns tuple of arrays, where the first indicates the row locations and the second indicates the column locations

def loadGeneratedDSM():

    # create a white image
    img = Image.new('RGB', (5000, 5000), color='white')

    # create an ImageDraw object
    draw = ImageDraw.Draw(img)
    rect_size_list =[10, 20, 50, 10, 20, 10, 20, 10, 20]
    gray_value_list = [0, 50, 100, 150, 200]

    # divide the image into 2500 equal squares (blocks)
    num_blocks = 50
    block_size = img.width // num_blocks
    dsm_blocks = [[0,0,0] for i in range(2500)]

    # draw random gray rectangles in each block
    for i in range(num_blocks):
        for j in range(num_blocks):
            # calculate the x and y coordinates of the top left corner of the block
            block_x1 = i * block_size
            block_y1 = j * block_size
            block_x2 = block_x1 + block_size
            block_y2 = block_y1 + block_size
            gray_value = random.choice(gray_value_list)
            dsm_blocks[50*i+j][0] = gray_value
            rect_size = random.choice(rect_size_list)
            dsm_blocks[50*i+j][1] = rect_size
            gap_size = random.randint(5, 10)
            dsm_blocks[50*i+j][2] = gap_size

            num_rect = (block_size - gap_size) // (rect_size + gap_size)

            # calculate the x and y coordinates of each rectangle in the block
            for k in range(num_rect):
                for l in range(num_rect):
                    rect_x1 = block_x1 + k * (rect_size + gap_size)
                    rect_y1 = block_y1 + l * (rect_size + gap_size)
                    rect_x2 = rect_x1 + rect_size
                    rect_y2 = rect_y1 + rect_size
                    
                    # draw a random gray rectangle in the block
                    color = (gray_value, gray_value, gray_value)
                    draw.rectangle((rect_x1, rect_y1, rect_x2, rect_y2), fill=color)

    points = []
    x_range = img.width // 10
    y_range = img.height // 10
    lines = [[0,0,0] for i in range(10)]
    for i in range(10):
        if i%2==0:
            x1 = 0
            y1 = random.randint(i*y_range, (i+1)*y_range - 1)
            x2 = img.width - 1
            y2 = random.randint(i*y_range, (i+1)*y_range - 1)
            points.append(((x1, y1), (x2, y2)))
            lines[i][0] = y1
            lines[i][1] = y2

        else: 
            x1 = random.randint(i*x_range, (i+1)*x_range - 1)
            y1 = 0
            x2 = random.randint(i*x_range, (i+1)*x_range - 1)
            y2 = img.height - 1
            points.append(((x1, y1), (x2, y2)))
            lines[i][0] = x1
            lines[i][1] = x2
    # create a copy of the original image for the second image
    img2 = img.copy()

    # draw white lines of random width between the chosen pairs of points
    count = 0
    for point in points:
        x1, y1 = point[0]
        x2, y2 = point[1]
        line_width = random.choice([100, 50, 70])
        lines[count][2] = line_width
        if count%2==0:
            draw.line((x1, y1, x2, y2), fill=(254,254,254), width=line_width)
            draw2 = ImageDraw.Draw(img2)
            draw2.line((x1, y1, x2, y2), fill=(254,254,254), width=line_width)
        else: 
            draw.line((x1, y1, x2, y2), fill=(253,253,253), width=line_width)
            draw2 = ImageDraw.Draw(img2)
            draw2.line((x1, y1, x2, y2), fill=(253,253,253), width=line_width)
        count+=1

    #creating a new array and converting colors to heights so we
    #get a dsm array
    img_arr = np.array(img)

    dsm_arr = np.zeros(shape=(5000,5000),dtype=float)
    zero_pixels = np.all(img_arr == [0, 0, 0], axis=2)
    dsm_arr[zero_pixels] = 45
    #includes only over-rooftops cases, so all building need to be below drones' height
    fifty_pixels = np.all(img_arr == [50, 50, 50], axis=2)
    dsm_arr[fifty_pixels] = 30
    one_hundred_pixels = np.all(img_arr == [100, 100, 100], axis=2)
    dsm_arr[one_hundred_pixels] = 20
    one_hundred_fifty_pixels = np.all(img_arr == [150, 150, 150], axis=2)
    dsm_arr[one_hundred_fifty_pixels] = 10
    two_hundred_pixels = np.all(img_arr == [200, 200, 200], axis=2)
    dsm_arr[two_hundred_pixels] = 5
    white_pixels = np.all(img_arr == [253, 253, 253], axis=2)
    dsm_arr[white_pixels] = 0
    white_pixels = np.all(img_arr == [254, 254, 254], axis=2)
    dsm_arr[white_pixels] = 0
    white_pixels = np.all(img_arr == [255, 255, 255], axis=2)
    dsm_arr[white_pixels] = 0
    

    # save the grayscale image
    img.save('gray_rectangles.png')
    img2.save('lines.png')
    return dsm_arr, dsm_blocks, lines, img_arr
