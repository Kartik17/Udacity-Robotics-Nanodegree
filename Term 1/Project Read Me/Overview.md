# Project 1: SEARCH AND SAMPLE RETURN

## OBSTACLE AND ROCK IDENTIFICATION IN JUPYTER NOTEBOOK

To identify the obstacles and rocks in a given image, I modified the given function color_thresh:
1. To store the threshold of the navigable, obstacle , and rocks I created three separate numpy array of zeros as the same size as that of the image.
2. Threshold value of RGB for navigable path, obstacle, and obstacle was selected to be (160,160,160), (140,140,140) and (125,125,50), with relational operations as shown in the notebook.
3. The zero numpy arrays defined in the first step is assigned with value of 1 for the pixels where the conditionals are satisfied.
4. For obstacle threshold, only pixels that lie within the angular range same as that of the warped image.

### Result:
