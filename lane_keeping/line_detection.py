import cv2
import numpy as np
import time
# from matplotlib import pyplot as plt
# from scipy.optimize import minimize
# from scipy import stats
# from ltsfit.ltsfit import ltsfit


def line_detection(image,set_width,cut_top):
    def auto_canny(image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)
        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)
        # return the edged image
        return edged    

    
    height, width, _ = image.shape
    height, width, _ = image.shape
    # print("Height: ", height, "Width: ", width)
    # Crop ROI and resize to 640px widt:
    image = image[int(height*cut_top):, :]
    image = cv2.resize(image, (set_width, int(height/width*set_width)), interpolation = cv2.INTER_AREA)


    height, width, _ = image.shape
    # print("Height: ", height, "Width: ", width)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # OPTIONAL: Apply Gaussian blur to reduce noise
    # gray = cv2.GaussianBlur(gray,(11,11),0)
 
    # Use Canny edge detection to find edges in the image
    edges = auto_canny(gray)
  
    
    # Normal Hough transform. Results in rho, theta
    lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=120)

    # Draw and classify all lines:
    left_rho_theta=[]
    right_rho_theta=[]
    # print(lines)



    if lines is not None:
        # print(lines)
        # break
        line_left = []
        line_right = []
        for line in lines:
            rho,theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            if theta < np.pi/3:
                line_left.append([x1, y1])
                line_left.append([x2, y2])
                cv2.line(image,(x1,y1),(x2,y2),(0,100,0),2)
                left_rho_theta.append([rho,theta])
            elif theta > 2*np.pi/3:
                line_right.append([x1, y1])
                line_right.append([x2, y2])
                cv2.line(image,(x1,y1),(x2,y2),(0,0,100),2)
                right_rho_theta.append([rho,theta])
            else:   # Not left or right lane:
                # cv2.line(image,(x1,y1),(x2,y2),(255,0,0),2)
                pass
    else:
        # print("Did not find any lines")
        return None
    
    if not (line_left and line_right):
        # print("Did NOT find BOTH lines")
        return None 
        
    
      
    # Define the function to optimize (sum of squared errors)
    def error_function(params, data):
        a, b = params
        x, y = data.T
        predicted = a * x + b
        error = np.sum((y - predicted)**2)
        return error

    def draw_line(a,b,rgb,width=2,image=image):
        cv2.line(image,(0,int(b)),(1000,int(a*1000+b)),rgb,width)

    def rho_theta_to_ab(rho, theta):
        if np.sin(theta) == 0:
            a = -np.cos(theta) / 0.001
            b = rho / 0.001
        else:
            a = -np.cos(theta) / np.sin(theta)
            b = rho / np.sin(theta)
        return a, b
 

    
    line_left = np.array(line_left)
    line_right = np.array(line_right)   

    # Line segments defined with star(x1, y1) and end(x2, y2)
    x1=line_left[:,0]
    y1=line_left[:,1]
    x2=line_right[:,0]
    y2=line_right[:,1]



    # Choose line fitting method:
    # -------------------------------------------

    # Median(PREFERED for hough transform):

    lines = np.array(left_rho_theta)
    rho = np.median(lines[:,0])
    theta = np.median(lines[:,1])
    a1,b1 = rho_theta_to_ab(rho,theta)
    draw_line(a1,b1,(0,0,200),3)


    lines = np.array(right_rho_theta)
    rho = np.median(lines[:,0])
    theta = np.median(lines[:,1])
    a2,b2 = rho_theta_to_ab(rho,theta)
    draw_line(a2,b2,(200,0,0),3)


    # print(a1,b1,a2,b2)
    # cv2.imshow("alle streker", image)
    # cv2.waitKey(0)
    # -------------------------------------------

    # # Least Squares Polyfit:
    # start = time.time()
    # a, b = np.polyfit(x1, y1, 1)
    # end = time.time()
    # tid_polyfit.append(end-start)
    # # draw_line(a,b,(0,100,0),8)

    # start = time.time()
    # a, b = np.polyfit(x2, y2, 1)
    # end = time.time()
    # tid_polyfit.append(end-start)
    # print("NEW method: ",a,b, " in ", end-start)
    # # draw_line(a,b,(0,100,0),8)
    
    # -------------------------------------------

    # # Least Squares Linregress:
    # start = time.time()
    # a, b, r_value, p_value, std_err = stats.linregress(x1,y1)
    # end = time.time()
    # tid_linregres.append(end-start)
    # # draw_line(a,b,(0,140,0),6)

    # start = time.time()
    # a, b, r_value, p_value, std_err = stats.linregress(x2,y2)
    # end = time.time()
    # tid_linregres.append(end-start)
    # print("NEWNEW method: ",a,b, " in ", end-start)
    # # draw_line(a,b,(0,140,0),6)
    
    # -------------------------------------------

    # # Least Squares Manual:
    # start = time.time()
    # xm=np.mean(x1)
    # ym=np.mean(y1)
    # a1=np.sum((x1-xm)*(y1-ym))/np.sum((x1-xm)**2)
    # b1=ym-(a*xm)
    # end = time.time()
    # # draw_line(a,b,(0,180,0),4)

    # start = time.time()
    # xm=np.mean(x2)
    # ym=np.mean(y2)
    # a2=np.sum((x2-xm)*(y2-ym))/np.sum((x2-xm)**2)
    # b2=ym-(a*xm)
    # end = time.time()


    # # print("MANUAL method: ",a,b, " in ", end-start)
    # draw_line(a1,b1,(0,180,0),4)
    # draw_line(a2,b2,(0,180,0),4)
    # cv2.imshow("alle streker", image)
    # cv2.waitKey(0)
    # -------------------------------------------

    # # Least Squares (minimize):
    # initial_params = [1, 1]

    # start = time.time()
    # result = minimize(error_function, initial_params, args=(line_left,))
    # end = time.time()
    # tid_ls.append(end-start)

    # optimal_params = result.x
    # a1_optimal, b1_optimal = optimal_params
    # print(f"Line: y = {a1_optimal:.2f}x + {b1_optimal:.2f} in {end-start:-2f}")
    # cv2.line(image,(0,int(b1_optimal)),(2000,int(a1_optimal*2000+b1_optimal)),(0,220,0),2)
    
    # start = time.time()
    # result = minimize(error_function, initial_params, args=(line_right,))
    # end = time.time()
    # tid_ls.append(end-start)

    # optimal_params = result.x
    # a2_optimal, b2_optimal = optimal_params
    # print(f"Line: y = {a2_optimal:.2f}x + {b2_optimal:.2f}")
    # cv2.line(image,(0,int(b2_optimal)),(2000,int(a2_optimal*2000+b2_optimal)),(0,220,0),2)

    # -------------------------------------------

    # Robust line fit

    # sigx1=np.ones_like(x1)
    # sigy1=np.ones_like(y1)
    # sigx2=np.ones_like(x2)
    # sigy2=np.ones_like(y2)
    # try:
    #     s=time.time()
    #     p = ltsfit(x1, y1, sigx1, sigy1, clip=2.6, corr=True, epsy=True,
    #             frac=None, label='Fitted', label_clip='Clipped',
    #             legend=True, pivot=None, plot=True, text=True)
    #     b1_optimal, a1_optimal = p.coef
    #     tid_robust.append(time.time()-s)
    #     print(f"Best fitting parameters: {p.coef} in {time.time()-s} seconds")
    #     cv2.line(image,(0,int(b1_optimal)),(2000,int(a1_optimal*2000+b1_optimal)),(0,0,0),4)

    #     s=time.time()
    #     p = ltsfit(x2, y2, sigx2, sigy2, clip=2.6, corr=True, epsy=True,
    #             frac=None, label='Fitted', label_clip='Clipped',
    #             legend=True, pivot=None, plot=True, text=True)
    #     b2_optimal, a2_optimal = p.coef
    #     tid_robust.append(time.time()-s)
    #     print(f"Best fitting parameters: {p.coef} in {time.time()-s} seconds")
    #     cv2.line(image,(0,int(b2_optimal)),(2000,int(a2_optimal*2000+b2_optimal)),(0,0,0),4)
    # except:     # Wierd bug that crashes sometimes
    #     pass
    # ------------------------------------------
    
    return [a1, b1, a2, b2]