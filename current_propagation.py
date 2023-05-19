
import surface as surface
import soldier as soldier
import drones as drones
import numpy as np

#based on 
#Recommendation  ITU-R  P.1411-6
#(02/2012)

#Propagation data and prediction methods
#for the planning of short-range outdoor radiocommunication systems and radio
#local area networks in the frequency
#range 300 MHz to 100 GHz

#here f in Mhz

c = 3*10**8 # speed of light, m/s
upsilon = 0.0417
chi = 0.1
avg_soldier_height = 1.5 #carries equipment on the back
drone_height = 50
block_size = 100

def los_path_loss(sold, surf, freq, drone_pos_loc):
    #distance between soldier and drone
    #hb - height of drone
    #hm - height of soldier
    #lambda - frequency of drone
    #TODO: figure out what the frequency should be - MERAV ANSWER: cellular frequency
    #we assume the wave velocity is at the speed of light
    drone_loc = drone_pos_loc
    dtm = surface.surface.getDTM(surf)
    hb =  dtm[drone_pos_loc]+drone_height
    sold_loc = soldier.soldier.getLocation(sold)
    hm = surface.surface.getDSM(surf)[sold_loc]+avg_soldier_height
    vertical_dist = np.square(hb-hm)
    horizontal_dist = np.square(drone_loc[0]-sold_loc[0])+np.square(drone_loc[1]-sold_loc[1])
    dist = np.sqrt(vertical_dist+horizontal_dist)
    horizontal_dist = np.sqrt(horizontal_dist)
    #print("h (in meters) = " +str(hb))
    #print("d(u,n) (in meters) = " +str(dist))
    wave_length = c/freq #m
    rbp = (4*hb*hm)/wave_length
    lbp = np.abs(20*np.log10((wave_length**2)/(8*np.pi*hb*hm)))
    if (dist<=rbp):
        coeff = 20
    else:
        coeff = 40
    return lbp+6+coeff*np.log10(dist/rbp)

def nlos_path_loss(sold, surf, freq,drone_pos_loc):
    drone_loc = drone_pos_loc
    dtm = surface.surface.getDTM(surf)
    hb =  dtm[drone_pos_loc]+drone_height
    sold_loc = soldier.soldier.getLocation(sold)
    hm = surface.surface.getDSM(surf)[sold_loc]+avg_soldier_height
    vertical_dist = np.square(hb-hm)
    horizontal_dist = np.square(drone_loc[0]-sold_loc[0])+np.square(drone_loc[1]-sold_loc[1])
    dist = np.sqrt(vertical_dist+horizontal_dist)
    horizontal_dist = np.sqrt(horizontal_dist)
    #street_width = surface.surface.
    # our world/map is straightened so we can just take 'regular' arctan
    phi = np.arctan2(np.abs(drone_loc[1]-sold_loc[1]),np.abs(drone_loc[0]-sold_loc[0]))
    if phi>90:
        phi -= 90
    L_bf = 32.4 + 20*np.log10(dist/1000) + 20*np.log10((freq/(10**6)))
    block_num = calcBlock(sold_loc)
    dsm_blocks = surface.surface.getDSMBlocks(surf)
    lines = surface.surface.getDSMLines(surf)
    img = surface.surface.getDSMImage(surf)
    # (gray_value,rect_size,gap_size)
    street_width = find_street_width(sold_loc,block_num,dsm_blocks,lines,img)
    hr = calc_hr(sold_loc, block_num, dsm_blocks,surf)
    b = calc_b(sold_loc, block_num, dsm_blocks)
    del_hm = hr-hm
    del_hb = hb-hr
    wave_length = c/freq
    ds = (wave_length*dist**2)/(del_hb**2)
    L_rts = calc_Lrts(street_width, freq, del_hm, phi)
    L_msd = calc_Lmsd(dist,horizontal_dist,ds,del_hb,wave_length,hb,hr,freq,b)
    if ((L_msd+L_rts)<=0):
        return L_bf
    return L_bf+L_rts+L_msd

def calc_Lrts(w,f,del_hm,phi):
    L_ori = calc_Lori(phi)
    f = f/(10**6) #convert to MHz
    return -8.2-10*np.log10(w)+10*np.log10(f)+20*np.log10(del_hm)+L_ori

def calc_Lmsd(d,l,ds,del_hb,lamda,hb,hr,f,b):
    #we calculate l as the horizontal distance since it only goes inside a log and thus
    #it does not make a big difference compared to the actual l
    dbp = np.abs(del_hb)*np.sqrt(l/lamda)
    l_upp = calc_L1_msd(dbp,hb,hr,del_hb,f,b) #this is L1_msd_dbp
    l_low = calc_L2_msd(dbp,del_hb,b,lamda,f,hb,hr) #this is L2_msd_dbp
    dh_bp = l_upp-l_low
    zeta = (l_upp-l_low)*upsilon
    l_mid = (l_upp+l_low)/2
    L1_msd_d = calc_L1_msd(d,hb,hr,del_hb,f,b)
    L2_msd_d = calc_L2_msd(d,del_hb,b,lamda,f,hb,hr)
    if (l>ds and dh_bp>0):
        return -np.tanh((np.log10(d)-np.log10(dbp))/chi)*(L1_msd_d-l_mid)+l_mid
    if (l<=ds and dh_bp>0):
        return np.tanh((np.log10(d)-np.log10(dbp))/chi)*(L2_msd_d-l_mid)+l_mid
    if (dh_bp==0):
        return L2_msd_d
    if (l>ds and dh_bp<0):
        return L1_msd_d-np.tanh((np.log10(d)-np.log10(dbp))/zeta)*(l_upp-l_mid)-l_upp+l_mid
    if (l<=ds and dh_bp<0):
        return L2_msd_d+np.tanh((np.log10(d)-np.log10(dbp))/zeta)*(l_mid-l_low)+l_mid-l_low
    return 0


def calc_Lori(phi):
    assert(0<=phi<=90)
    if (0<=phi<35):
        return -10*0.354*phi
    elif (35<=phi<55):
        return 2.5+0.075*(phi-35)
    else: # 55<=phi<=90
        return 4-0.114*(phi-55)
    
def calcBlock(sold_loc):
    x,y = sold_loc
    return (x//100)+50*(y//100)

def find_street_width(sold_loc,block_num, dsm_blocks,lines,img):
    x, y = sold_loc
    block_len = 100
    gray_value, rect_size, gap_size = dsm_blocks[block_num]
    if in_line(x, y, img):
        line_num = find_line(x, y, img)
        line_width = lines[line_num][2]
        #if we're in a line, than basically we are in a street (created by the line)
        #therefore, the street width in this case would be the line width
        #if we're in an intersection between lines, then the latest one to was added to the image
        #will be chosen
        return line_width
    border = calc_border(rect_size,gap_size)
    # if on boarder, than x = 49 or y = 49 for border = 50 for example
    if (x%block_len<border and y%block_len<border):
        # the soldier is inside the 'complex'
        return gap_size
    # the soldier is OUTSIDE the 'complex' in some manner
    x_rounded_to_100_multiplier = int(np.ceil(x/100.0)*100)
    y_rounded_to_100_multiplier = int(np.ceil(x/100.0)*100)
    #special case
    if (x_rounded_to_100_multiplier==5000 or y_rounded_to_100_multiplier==5000):
        return block_len-border
    #we are checking if we're on the right side of the white gap or on the lower side
    from_right = False
    if (x%block_len>=block_len-border):
        from_right = True
    from_down = False
    if (y%block_len>=block_len-border):
        from_down = True
    right_width = 0
    if (from_right):
        if in_line(x_rounded_to_100_multiplier,y,img):
            line_num = find_line(x_rounded_to_100_multiplier,y,img)
            line_width = lines[line_num][2]
            right_width = block_len-border+line_width
        else:
            right_width = block_len-border
    down_width = 0
    if (from_down):
        if in_line(x,y_rounded_to_100_multiplier,img):
            line_num = find_line(x,y_rounded_to_100_multiplier,img)
            line_width = lines[line_num][2]
            down_width = block_len-border+line_width
        else:
            down_width = block_len-border
    if (right_width==0): #not from_right
        return down_width
    elif (down_width==0): #not from_down
        return right_width
    return (down_width+right_width)/2 #average because from_right + from_down



def in_line(x, y, img):
    #we know that all triplets in the dsm_image contain the same value
    #so we can just slice on the first index
    if img[x,y,0]==254 or img[x,y,0]==253:
        return True
    return False

def find_line(x, y, img):
    if img[x,y,0]==254:
        # according to range
        line = y//500
        return line
    else:
        line = x//500
    return line

def calc_border(rect_size, gap_size):
    match rect_size:
        case 50:
            return 50
        case 20:
            return 3*rect_size + 2*gap_size
        case 10:
            if (gap_size==5):
                #6 rectangles in a row
                return 6*rect_size + 5*gap_size
            elif (6<=gap_size<=8):
                #5 rectangles
                return 5*rect_size + 4*gap_size
            else: #4 rectangles in a row
                return 4*rect_size + 3*gap_size

def calc_hr(sold_loc, block_num, dsm_blocks, surf):
    x, y = sold_loc
    block_len = 100
    gray_value, rect_size, gap_size = dsm_blocks[block_num]
    border = calc_border(rect_size,gap_size)
    block_init_x = block_size*(block_num%50)
    block_init_y = block_size*(block_num//50)
    block_height = surface.surface.getDSM(surf)[block_init_x,block_init_y]
    #block_height = from_gray_to_height(gray_value)
    if (x%block_len<border and y%block_len<border):
        return block_height
    right_height = -1
    down_height = -1
    if (x%block_len>=block_len-border and block_num%50!=49): #block not in rightmost column
        right_gray_value, right_rect_size, right_gap_size = dsm_blocks[block_num+1]
        block_init_x = block_size*((block_num+1)%50)
        block_init_y = block_size*((block_num+1)//50)
        right_height = surface.surface.getDSM(surf)[block_init_x,block_init_y]
        #right_height = from_gray_to_height(right_gray_value)
    if (y%block_len>=block_len-border and block_num<2450): #block not in last row
        down_gray_value, down_rect_size, down_gap_size = dsm_blocks[block_num+50]
        block_init_x = block_size*((block_num+50)%50)
        block_init_y = block_size*((block_num+50)//50)
        down_height = surface.surface.getDSM(surf)[block_init_x,block_init_y]
        #down_height = from_gray_to_height(down_gray_value)
    if (right_height==-1): #only need to average down_height
        return (block_height+down_height)/2
    if (down_height==-1): #only need to average right_height
        return (block_height+right_height)/2
    #need to average the 3 of them and the down right one
    down_right_gray_value, down_right_rect_size, down_right_gap_size = dsm_blocks[block_num+51]
    #down_right_height = from_gray_to_height(down_right_gray_value)
    block_init_x = block_size*((block_num+51)%50)
    block_init_y = block_size*((block_num+51)//50)
    down_right_height = surface.surface.getDSM(surf)[block_init_x,block_init_y]
    return (block_height+down_height+right_height+down_right_height)/4

def calc_b(sold_loc, block_num, dsm_blocks):
    #similar to calc_hr
    #b is the average building separation so if a soldier is inside
    #the complex the average building separation will be the gap_size+rect_size
    #if the soldier is outside the complex, the average building separation will be 
    # the average of the 2 or 4 blocks' b's to the right and downwards (including the block itself)
    x, y = sold_loc
    block_len = 100
    gray_value, rect_size, gap_size = dsm_blocks[block_num]
    border = calc_border(rect_size,gap_size)
    if (x%block_len<border and y%block_len<border):
        return gap_size+rect_size
    right_side = -1
    down_side = -1
    if (x%block_len>=block_len-border and block_num%50!=49): #block not in rightmost column
        right_gray_value, right_rect_size, right_gap_size = dsm_blocks[block_num+1]
        right_side = right_rect_size+right_gap_size
    if (y%block_len>=block_len-border and block_num<2450): #block not in last row
        down_gray_value, down_rect_size, down_gap_size = dsm_blocks[block_num+50]
        down_side = down_rect_size + down_gap_size
    if (right_side==-1): #only need to average down_height
        return (gap_size+rect_size+down_side)/2
    if (down_side==-1): #only need to average right_height
        return (gap_size+rect_size+right_side)/2
    #need to average the 3 of them and the down right one
    down_right_gray_value, down_right_rect_size, down_right_gap_size = dsm_blocks[block_num+51]
    return (gap_size+rect_size+right_side+down_side+down_right_rect_size+down_right_gap_size)/4

def calc_L1_msd(dbp,hb,hr,del_hb,f,b):
    L_bsh = calc_L_bsh(del_hb,hb,hr)
    k_a = calc_k_a(del_hb, dbp, hb, hr, f)
    k_d = calc_k_d(del_hb,hb,hr)
    k_f = calc_k_f(f)
    f = f/(10**6) #convert to MHz
    return L_bsh+k_a+k_d*np.log10(dbp/1000)+k_f*np.log10(f)-9*np.log10(b)

def calc_L_bsh(del_hb,hb,hr):
    if hb<=hr:
        return 0
    return -18*np.log10(1+del_hb)

def calc_k_a(del_hb, dbp, hb, hr, f):
    f_th = 2000*10**6
    if (hb>hr and f>f_th):
        return 71.4
    if (hb<=hr and f>f_th and dbp>=500):
        return 73-0.8*del_hb
    if (hb<=hr and f>f_th and dbp<500):
        return 73-1.6*del_hb*(dbp/1000)
    if (hb>hr and f<=f_th):
        return 54
    if (hb<=hr and f<=f_th and dbp>=500):
        return 54-0.8*del_hb
    if (hb<=hr and f<=f_th and dbp<500):
        return 54-1.6*del_hb*(dbp/1000)
    
def calc_k_d(del_hb,hb,hr):
    if (hb>hr):
        return 18
    return 18-15*(del_hb/hr)

def calc_k_f(f):
    f_th = 2000*10**6
    if f>f_th:
        return -8
    f = f/(10**6) #convert to MHz
    return -4+1.5*((f/925)-1)

def calc_L2_msd(dbp,del_hb,b,lamda,f,hb,hr):
    Q_m = calc_Q_m(dbp,del_hb,b,lamda,f,hb,hr)
    return -10*np.log10(Q_m**2)

def calc_Q_m(dbp,del_hb,b,lamda,f,hb,hr):
    print("dbp: "+str(dbp))
    print("del_hb: "+str(del_hb))
    print("b: "+str(b))
    print("lamda: "+str(lamda))
    print("f: "+str(f))
    print("hb: "+str(hb))
    print("hr: "+str(hr))
    delhu = calc_delhu(b,lamda, dbp)
    print("delhu: " +str(delhu))
    delhl = calc_delhl(b,f)
    print("delhl: " +str(delhl))
    theta = np.arctan2(del_hb/b,1)
    print("theta: "+str(theta))
    rho = np.sqrt(b**2+del_hb**2)
    if (hb>hr+delhu):
        return 2.35*((del_hb/dbp)*np.sqrt(b/lamda))**0.9
    if (hb<=hr+delhu and hb>=hr+delhl):
        return b/dbp
    if (hb<hr+delhl):
        (b/2*np.pi*dbp)*np.sqrt(lamda/rho)*((1/theta)-1/(2*np.pi+theta))

def calc_delhu(b,lamda, dbp):
    return 10**(-np.log10(np.sqrt(b/lamda))-(np.log10(dbp)/9)+(10/9)*np.log10(b/2.35))

def calc_delhl(b,f):
    f = f/(10**6) #convert to MHz
    numerator = 0.00023*b**2-0.1827*b-9.4978
    denominator = (np.log10(f))**2.938
    return (numerator/denominator) + 0.000781*b + 0.06923

def from_gray_to_height(gray):
    match gray:
        case 0:
            return 45
        case 50:
            return 30
        case 100:
            return 20
        case 150:
            return 10
        case 200:
            return 5
        case 253:
            return 0
        case 254:
            return 0
        case 255:
            return 0
    