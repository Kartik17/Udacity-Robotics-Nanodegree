import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    nav_thresh = np.zeros_like(img[:,:,0])
    obs_thresh = np.zeros_like(img[:,:,0])
    rock_thresh = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh_nav = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])

    below_thresh_obs = (img[:,:,0] < 120) \
                & (img[:,:,1] < 120) \
                & (img[:,:,2] < 120)

    rock_pixel = (img[:,:,0] > 125) \
                & (img[:,:,1] > 125) \
                & (img[:,:,2] < 50)

    # Index the array of zeros with the boolean array and set to 1
    nav_thresh[above_thresh_nav] = 1
    obs_thresh[below_thresh_obs] = 1
    rock_thresh[rock_pixel] = 1
    # Return the binary image
    return nav_thresh,obs_thresh,rock_thresh

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0],M,(img.shape[1], img.shape[0])))

    
    return warped, mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img

    image = Rover.img
    dst_size = 5
    bottom_offset = 6
    world_size = Rover.world_map[0]


    # 1) Define source and destination points for perspective transform
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]]) 

    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform

    warped, mask = perspect_transform(image,src,destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    nav_thresh, obs_thresh, rock_thresh = color_thresh()
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:,:,0] = obs_thresh
    Rover.vision_image[:,:,1] = rock_thresh
    Rover.vision_image[:,:,2] = nav_thresh

    # 5) Convert map image pixel values to rover-centric coords
    x_nav_rover, y_nav_rover = rover_coords(nav_thresh)
    x_obs_rover, y_obs_rover = rover_coords(obs_thresh)
    x_rock_rover, y_rock_rover = rover_coords(rock_thresh)

    # 6) Convert rover-centric pixel values to world coordinates
    xpos, ypos = Rover.pos[0], Rover.pos[1]
    yaw = Rover.yaw
    scale = 2*dst_size
    x_world_rover_nav, y_world_rover_nav = pix_to_world(x_nav_rover,y_nav_rover,xpos,ypos,yaw,world_size,scale) 
    x_world_rover_obs, y_world_rover_obs = pix_to_world(x_obs_rover,y_obs_rover,xpos,ypos,yaw,world_size,scale)
    x_world_rover_rock, y_world_rover_rock = pix_to_world(x_rock_rover,y_rock_rover,xpos,ypos,yaw,world_size,scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    Rover.worldmap[y_world_rover_obs,x_world_rover_obs,0] += 1
    Rover.worldmap[y_world_rover_rock,x_world_rover_rock,1] += 1
    Rover.worldmap[y_world_rover_rock,x_world_rover_rock,0],Rover.worldmap[y_world_rover_rock,x_world_rover_rock,1],data.worldmap[y_world_rover_rock,x_world_rover_rock,2] =255,255,255

    # 8) Convert rover-centric pixel positions to polar coordinates

    dist, angles = to_polar_coords(x_nav_rover,y_nav_rover)
    Rover.nav_dists = dist
    Rover.nav_angles = angles

    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    if x_rock_rover:
        Rover.near_sample = 1
        rock_dist,rock_angles = to_polar_coords(x_rock_rover,y_rock_rover)
        Rover.rock_dist = rock_dist
        Rover.rock_angles = rock_angles

 
    
    
    return Rover