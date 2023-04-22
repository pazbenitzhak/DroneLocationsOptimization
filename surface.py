import rasterio
import numpy as np
import blocks as blocks
import random
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

Image.MAX_IMAGE_PIXELS = 100000000

class surface:

    def __init__(self, dtm_path,sold_th,drone_th):
        # blocks - cluster of lists of valid locations for soldiers
        # drone_map - matrix of valid locations for drones where 1 = valid
        self.dsm, self.dsm_blocks, self.dsm_city_grid, self.dtm, self.diffs_dsm_dtm, \
            self.blocks, self.white_block_indexes, self.drone_map = loadSurface(dtm_path,sold_th,drone_th)

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

def loadSurface(dtm_path,sold_th, drone_th):
    """dsm_dataset = rasterio.open(dsm_path)
    dsm = dsm_dataset.read()
    dsm = dsm[0][0:5000,5000:]"""
    dsm, dsm_blocks, dsm_city_grid = loadGeneratedDSM()
    dtm_dataset = rasterio.open(dtm_path)
    dtm = dtm_dataset.read()
    dtm = dtm[0][0:5000,5000:]
    diffs = dsm-dtm
    """cond = (diffs<=sold_th).astype(int)
    blocks_array = blocks.block.classifyRouteBlocks(cond)"""
    indices = np.where(dsm==0)
    # a list of list because there should be backwards compatibility
    white_block_indexes = [list(zip(indices[0], indices[1]))] #there is only one block in the new DSM implementation
    white_block = (dsm==0)
    drone_map = (diffs<=drone_th).astype(int)
    drone_map_zeros = np.argwhere(drone_map==0)
    for ind in drone_map_zeros:
        for x_add in range(-10,11):
            for y_add in range(-10,11):
                if (0<=ind[0]+x_add<5000 and 0<=ind[1]+y_add<5000) and (drone_map[ind[0]+x_add,ind[1]+y_add]==0) \
                    and ((x_add)**2 + (y_add)**2 <= 10**2):
                    drone_map[ind[0]+x_add,ind[1]+y_add] = 0
                    #TODO: for presentation
    return dsm, dsm_blocks, dsm_city_grid, dtm, diffs, white_block, white_block_indexes, drone_map

def findSoldiersValidLocations(diffs, threshold):
    condition = (diffs<=threshold).astype(int)
    validLoc = np.where(condition==1)
    return validLoc #returns tuple of arrays, where the first indicates the row locations and the second indicates the column locations

def loadGeneratedDSM():

    # create a white image
    img = Image.new('RGB', (5000, 5000), color='white')

    # create an ImageDraw object
    draw = ImageDraw.Draw(img)

    # define the size of the rectangles and the gap between them
    rect_size_list =[10, 20, 50]
    gray_value_list = [0, 50, 100, 150, 200, 50, 100, 150, 200]
    #gap_size = 10

    # divide the image into 2500 equal squares (blocks)
    num_blocks = 50
    block_size = img.width // num_blocks
    dsm_blocks = [[0,0,0] for i in range(2500)]
    # (gray_value,rect_size,gap_size)
    # calculate the number of rectangles that fit in a row/column of a block
    #num_rect = (block_size - gap_size) // (rect_size + gap_size)

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
            #gray_value = 240
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

    # choose 10 random pairs of points
    ##points = []
    ##for i in range(10):
    ##    if i%2==0:
    ##        x1 = 0
    ##        y1 = random.randint(0, img.height - 1)
    ##        x2 = img.width - 1
    ##        y2 = random.randint(0, img.height - 1)
    ##        points.append(((x1, y1), (x2, y2)))
    ##    else: 
    ##        x1 = random.randint(0, img.width - 1)
    ##        y1 = 0
    ##        x2 = random.randint(0, img.width - 1)
    ##        y2 = img.height - 1
    ##        points.append(((x1, y1), (x2, y2)))
    ##
    points = []
    x_range = img.width // 10
    y_range = img.height // 10
    lines = [[0,0,0] for i in range(10)]
    #lines: (x1,x2,width) or (y1,y2,width) - depending on oddity/even
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
        draw.line((x1, y1, x2, y2), fill='white', width=line_width)
        draw2 = ImageDraw.Draw(img2)
        draw2.line((x1, y1, x2, y2), fill='white', width=line_width)
        count+=1
    
    ##
    ### Define the parameters for the rectangles
    ##num_rectangles = 10
    ##min_size = 200
    ##max_size = 800
    ##min_pos = 0
    ##max_pos = 4900
    ##
    ### Create an ImageDraw object
    ##draw = ImageDraw.Draw(img)
    ##
    ### Add the rectangles to the image
    ##rectangles = []
    ##for i in range(num_rectangles):
    ##    # Generate random parameters for the rectangle
    ##    x1 = random.randint(min_pos, max_pos)
    ##    y1 = random.randint(min_pos, max_pos)
    ##    width = random.randint(min_size, max_size)
    ##    height = random.randint(min_size, max_size)
    ##    x2 = x1 + width
    ##    y2 = y1 + height
    ##    
    ##    # Check for overlap with existing rectangles
    ##    overlaps = True
    ##    while overlaps:
    ##        overlaps = False
    ##        for rect in rectangles:
    ##            if x1 < rect[2] and x2 > rect[0] and y1 < rect[3] and y2 > rect[1]:
    ##                overlaps = True
    ##                x1 = random.randint(min_pos, max_pos)
    ##                y1 = random.randint(min_pos, max_pos)
    ##                width = random.randint(min_size, max_size)
    ##                height = random.randint(min_size, max_size)
    ##                x2 = x1 + width
    ##                y2 = y1 + height
    ##    
    ##    # Add the rectangle to the list of rectangles
    ##    rectangles.append((x1, y1, x2, y2))
    ##    
    ##    # Draw the rectangle on the image
    ##    draw.rectangle((x1, y1, x2, y2), fill=(255, 255, 255))
    ##
    ### Save the modified image
    ##img.save('modified_image.jpg')
    # save the images

    #creating a new array and converting colors to heights so we
    #get a dsm array
    img_arr = np.array(img)

    dsm_arr = np.zeros(shape=(5000,5000))

    zero_pixels = np.all(img_arr == [0, 0, 0], axis=2)
    dsm_arr[zero_pixels] = 50
    fifty_pixels = np.all(img_arr == [50, 50, 50], axis=2)
    dsm_arr[fifty_pixels] = 30
    one_hundred_pixels = np.all(img_arr == [100, 100, 100], axis=2)
    dsm_arr[one_hundred_pixels] = 20
    one_hundred_fifty_pixels = np.all(img_arr == [150, 150, 150], axis=2)
    dsm_arr[one_hundred_fifty_pixels] = 10
    two_hundred_pixels = np.all(img_arr == [200, 200, 200], axis=2)
    dsm_arr[two_hundred_pixels] = 5
    white_pixels = np.all(img_arr == [255, 255, 255], axis=2)
    dsm_arr[white_pixels] = 0


    # convert the RGB values to grayscale values using the luminosity method
    """gray = np.dot(arr[...,:3], [0.21, 0.72, 0.07])

    # normalize the grayscale values to the range [0, 255]
    gray = (gray / np.max(gray)) * 255

    # convert the grayscale values to integers
    gray_data_array = gray.astype(np.uint8)

    dsm_array = """
    # save the grayscale image
    img.save('gray_rectangles.png')
    img2.save('lines.png')
    return dsm_arr, dsm_blocks, lines
