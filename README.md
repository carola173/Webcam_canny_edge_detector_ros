# Webcam_canny_edge_detector_ros
Canny edge detection implementation on live camera feed as a input  using ROS and OpenC

Launch file : canny_ros.launch
Input : Webcam data
Output : Display the detected edges in the image
 
Canny Edge Detection in OpenCv implemets the following code inside it:

1.The Gaussian filter - This filter implements the blur removes some of the noise before further processing the image. Variable sigma is used indicating how much to blur the input image

2.The Sobel operator is applied and identifying the Gradient direction - Edge is place when the color of an image changes, hence the intensity of the pixel changes as well. This can be computed using the Sobel operator.

                                 - Magniture G= Sqrt(sq(Gx)+sq(Gy))
                                 - Angle G = arctan(Gx/Gy)
                                   

3. Applying the Non-Maximum Suppression -Non maximum suppression works by finding the pixel with the maximum value in an edge.  it occurs when pixel q has an intensity that is larger than both p and r where pixels p and r are the pixels in the gradient direction of q. If this condition is true, then we keep the pixel, otherwise we set the pixel to zero 

Non maximum suppression can be achieved by interpolating the pixels for greater accuracy:
                                - r=alpha*b+(1-alpha)*a 
                                
                                          
4. Hysteresis Filtering -Edge tracking is implement in this filter where the weak edges that are connected to strong edges will be actual/real edges. Weak edges that are not connected to strong edges will be removed. To speed up this process, my algorithm keeps track of the weak and strong edges that way I can recursively iterate through the strong edges and see if there are connected weak edges instead of having to iterate through every pixel in the image


** I have tried implementing all the steps, rather than using the openCv function directly detect_edges(frame)
