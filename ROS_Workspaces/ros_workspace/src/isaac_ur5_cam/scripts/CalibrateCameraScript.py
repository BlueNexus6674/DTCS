import numpy as np
import cv2 as cv2
import glob
from tqdm import tqdm
import yaml
#pip install yaml
#pip install tqdm
print("")
print("Starting Setup...")

distortion = "plumb_bob"

CheckerboardX = 8
CheckerboardY = 6
CheckerboardTotal = CheckerboardX * CheckerboardY
Checkerboard = (CheckerboardX, CheckerboardY)
world_scale = 0.035

# Set the path to the images captured by the left and right cameras
pathL = "./CalibData/Live/Img/"
pathR = "./CalibData/Live/Img/"
totalImages = 27
 
# Termination criteria for refining the detected corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
objp = np.zeros((CheckerboardTotal,3), np.float32)
objp[:,:2] = np.mgrid[0:CheckerboardX,0:CheckerboardY].T.reshape(-1,2)
objp = world_scale*objp
 
img_ptsL = []
img_ptsR = []
obj_pts = []

print("Starting Img Calculations...")

for imgNum in tqdm(range(1, totalImages+1)):
  imgL = cv2.imread(pathL+"left-%s.png" %(str(imgNum)).zfill(4))
  imgR = cv2.imread(pathR+"right-%s.png" %(str(imgNum)).zfill(4))
  imgL_gray = cv2.imread(pathL+"left-%s.png" %(str(imgNum)).zfill(4), 0)
  imgR_gray = cv2.imread(pathR+"right-%s.png" %(str(imgNum)).zfill(4), 0)
 
  outputL = imgL.copy()
  outputR = imgR.copy()
 
  retR, cornersR =  cv2.findChessboardCorners(outputR, Checkerboard, None)
  retL, cornersL = cv2.findChessboardCorners(outputL, Checkerboard, None)
 
  if retR and retL:
    obj_pts.append(objp)
    cv2.cornerSubPix(imgR_gray,cornersR, (11,11), (-1,-1), criteria)
    cv2.cornerSubPix(imgL_gray,cornersL, (11,11), (-1,-1), criteria)
    cv2.drawChessboardCorners(outputR, Checkerboard, cornersR, retR)
    cv2.drawChessboardCorners(outputL, Checkerboard, cornersL, retL)
    cv2.imshow('cornersR', outputR)
    cv2.imshow('cornersL', outputL)
    cv2.waitKey(0)
 
    img_ptsL.append(cornersL)
    img_ptsR.append(cornersR)
 
print("Img Calculations Complete")
print("")
print("Calibrating Left/Right Cameras...")
 
# Calibrating left camera
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj_pts,img_ptsL,imgL_gray.shape[::-1],None,None)
hL,wL= imgL_gray.shape[:2]
new_mtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),1,(wL,hL))
 
# Calibrating right camera
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj_pts,img_ptsR,imgR_gray.shape[::-1],None,None)
hR,wR= imgR_gray.shape[:2]
new_mtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),1,(wR,hR))





#---------------------------------------------------------------------------------------
print("Calibrating Stereo...")

flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same 
 
criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
 
# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retS, new_mtxL, distL, new_mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(obj_pts, img_ptsL, img_ptsR, new_mtxL, distL, new_mtxR, distR, imgL_gray.shape[::-1], criteria_stereo, flags)





#---------------------------------------------------------------------------
print("Calculating Rectification...")

rectify_scale= 1
rect_L, rect_R, proj_mat_L, proj_mat_R, Q, roiL, roiR= cv2.stereoRectify(new_mtxL, distL, new_mtxR, distR, imgL_gray.shape[::-1], Rot, Trns, rectify_scale,(0,0))





#----------------------------------
print("Calculating StereoMaps...")

Left_Stereo_Map= cv2.initUndistortRectifyMap(new_mtxL, distL, rect_L, proj_mat_L,
                                             imgL_gray.shape[::-1], cv2.CV_16SC2)
Right_Stereo_Map= cv2.initUndistortRectifyMap(new_mtxR, distR, rect_R, proj_mat_R,
                                              imgR_gray.shape[::-1], cv2.CV_16SC2)
 
print("")
print("Saving OpenCV Params ......")
print("")
cv_file = cv2.FileStorage("./CalibData/Calibration.xml", cv2.FILE_STORAGE_WRITE)
cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map[0])
cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map[1])
cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map[0])
cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map[1])
cv_file.release()





#-------------------- ROS YAML

print("ROS Parsing...")

L_Cam_Matrix_Data1 = ((str(new_mtxL[0]).split("[")[1]).split("]")[0])
L_Cam_Matrix_Data1 = ", ".join(L_Cam_Matrix_Data1.split())
L_Cam_Matrix_Data2 = ((str(new_mtxL[1]).split("[")[1]).split("]")[0])
L_Cam_Matrix_Data2 = ", ".join(L_Cam_Matrix_Data2.split())
L_Cam_Matrix_Data3 = ((str(new_mtxL[2]).split("[")[1]).split("]")[0])
L_Cam_Matrix_Data3 = ", ".join(L_Cam_Matrix_Data3.split())

L_Dis_Matrix_Data1 = (", ".join(str(distL[0]).split())).split(", ", 1)[1] 

L_Rec_Matrix_Data1 = ((str(rect_L[0]).split("[")[1]).split("]")[0])
L_Rec_Matrix_Data1 = ", ".join(L_Rec_Matrix_Data1.split())
L_Rec_Matrix_Data2 = ((str(rect_L[1]).split("[")[1]).split("]")[0])
L_Rec_Matrix_Data2 = ", ".join(L_Rec_Matrix_Data2.split())
L_Rec_Matrix_Data3 = ((str(rect_L[2]).split("[")[1]).split("]")[0])
L_Rec_Matrix_Data3 = ", ".join(L_Rec_Matrix_Data3.split())

L_Proj_Matrix_Data1 = ((str(proj_mat_L[0]).split("[")[1]).split("]")[0])
L_Proj_Matrix_Data1 = ", ".join(L_Proj_Matrix_Data1.split())
L_Proj_Matrix_Data2 = ((str(proj_mat_L[1]).split("[")[1]).split("]")[0])
L_Proj_Matrix_Data2 = ", ".join(L_Proj_Matrix_Data2.split())
L_Proj_Matrix_Data3 = ((str(proj_mat_L[2]).split("[")[1]).split("]")[0])
L_Proj_Matrix_Data3 = ", ".join(L_Proj_Matrix_Data3.split())



R_Cam_Matrix_Data1 = ((str(new_mtxR[0]).split("[")[1]).split("]")[0])
R_Cam_Matrix_Data1 = ", ".join(R_Cam_Matrix_Data1.split())
R_Cam_Matrix_Data2 = ((str(new_mtxR[1]).split("[")[1]).split("]")[0])
R_Cam_Matrix_Data2 = ", ".join(R_Cam_Matrix_Data2.split())
R_Cam_Matrix_Data3 = ((str(new_mtxR[2]).split("[")[1]).split("]")[0])
R_Cam_Matrix_Data3 = ", ".join(R_Cam_Matrix_Data3.split())

R_Dis_Matrix_Data1 = (", ".join(str(distR[0]).split())).split(", ", 1)[1] 

R_Rec_Matrix_Data1 = ((str(rect_R[0]).split("[")[1]).split("]")[0])
R_Rec_Matrix_Data1 = ", ".join(R_Rec_Matrix_Data1.split())
R_Rec_Matrix_Data2 = ((str(rect_R[1]).split("[")[1]).split("]")[0])
R_Rec_Matrix_Data2 = ", ".join(R_Rec_Matrix_Data2.split())
R_Rec_Matrix_Data3 = ((str(rect_R[2]).split("[")[1]).split("]")[0])
R_Rec_Matrix_Data3 = ", ".join(R_Rec_Matrix_Data3.split())

R_Proj_Matrix_Data1 = ((str(proj_mat_R[0]).split("[")[1]).split("]")[0])
R_Proj_Matrix_Data1 = ", ".join(R_Proj_Matrix_Data1.split())
R_Proj_Matrix_Data2 = ((str(proj_mat_R[1]).split("[")[1]).split("]")[0])
R_Proj_Matrix_Data2 = ", ".join(R_Proj_Matrix_Data2.split())
R_Proj_Matrix_Data3 = ((str(proj_mat_R[2]).split("[")[1]).split("]")[0])
R_Proj_Matrix_Data3 = ", ".join(R_Proj_Matrix_Data3.split())

print ("Writing to ROS Calibration Files...")


file = open("./CalibData/ROSLeft.yaml", 'w')
file.write("")
file.close()
	
file = open("./CalibData/ROSLeft.yaml", 'a')
file.write("image_width: " + str(wL) + "\n")
file.write("image_height: " + str(hL) + "\n")
file.write("camera_name: left \n")
file.write("camera_matrix: \n")
file.write("  rows: 3 \n")
file.write("  cols: 3 \n")
file.write("  data: [ " + L_Cam_Matrix_Data1 + ",\n")
file.write("        " + L_Cam_Matrix_Data2 + ",\n")
file.write("        " + L_Cam_Matrix_Data3 + "]\n")
file.write("distortion_model: " + distortion + " \n")
file.write("distortion_coefficients: \n")
file.write("  rows: 1 \n")
file.write("  cols: 5 \n")
file.write("  data: [" + L_Dis_Matrix_Data1 + "\n")
file.write("rectification_matrix: \n")
file.write("  rows: 3 \n")
file.write("  cols: 3 \n")
file.write("  data: [ " + L_Rec_Matrix_Data1 + ",\n")
file.write("        " + L_Rec_Matrix_Data2 + ",\n")
file.write("        " + L_Rec_Matrix_Data3 + "]\n")
file.write("projection_matrix: \n")
file.write("  rows: 3 \n")
file.write("  cols: 4 \n")
file.write("  data: [ " + L_Proj_Matrix_Data1 + ",\n")
file.write("        " + L_Proj_Matrix_Data2 + ",\n")
file.write("        " + L_Proj_Matrix_Data3 + "]\n")
file.close()

print ("ROS Left Complete...")

file = open("./CalibData/ROSRight.yaml", 'w')
file.write("")
file.close()
	
file = open("./CalibData/ROSRight.yaml", 'a')
file.write("image_width: " + str(wR) + "\n")
file.write("image_height: " + str(hR) + "\n")
file.write("camera_name: right \n")
file.write("camera_matrix: \n")
file.write("  rows: 3 \n")
file.write("  cols: 3 \n")
file.write("  data: [ " + R_Cam_Matrix_Data1 + ",\n")
file.write("        " + R_Cam_Matrix_Data2 + ",\n")
file.write("        " + R_Cam_Matrix_Data3 + "]\n")
file.write("distortion_model: " + distortion + " \n")
file.write("distortion_coefficients: \n")
file.write("  rows: 1 \n")
file.write("  cols: 5 \n")
file.write("  data: [" + R_Dis_Matrix_Data1 + "\n")
file.write("rectification_matrix: \n")
file.write("  rows: 3 \n")
file.write("  cols: 3 \n")
file.write("  data: [ " + R_Rec_Matrix_Data1 + ",\n")
file.write("        " + R_Rec_Matrix_Data2 + ",\n")
file.write("        " + R_Rec_Matrix_Data3 + "]\n")
file.write("projection_matrix: \n")
file.write("  rows: 3 \n")
file.write("  cols: 4 \n")
file.write("  data: [ " + R_Proj_Matrix_Data1 + ",\n")
file.write("        " + R_Proj_Matrix_Data2 + ",\n")
file.write("        " + R_Proj_Matrix_Data3 + "]\n")
file.close()

print ("ROS Right Complete...")
print ("ROS Write Complete")
print("") 

print("Starting ReProjection Error Right")
mean_error = 0
for i in range(len(obj_pts)):
	imgpoints2, _ = cv2.projectPoints(obj_pts[i], rvecsL[i], tvecsL[i], new_mtxL, distL)
	error = cv2.norm(img_ptsL[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
	mean_error += error
print( "total error: {}".format(mean_error/len(obj_pts)) )
print("")

print("Starting ReProjection Error Right")
mean_error = 0
for i in range(len(obj_pts)):
	imgpoints2, _ = cv2.projectPoints(obj_pts[i], rvecsR[i], tvecsR[i], new_mtxR, distR)
	error = cv2.norm(img_ptsR[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
	mean_error += error
print( "total error: {}".format(mean_error/len(obj_pts)) )
print("")
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

