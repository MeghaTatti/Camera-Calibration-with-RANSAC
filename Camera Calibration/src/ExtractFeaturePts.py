import cv2
import numpy as np
import glob


def main():

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    coordinates = np.zeros((6*7,3), np.float32)
    coordinates[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    #coordinates = np.mgrid[0:7,0:6].T.reshape(-1,2)*40
    points_3D = []
    points_2D = []

    images = glob.glob('*.jpg')

    for f_name in images:
        image = cv2.imread(f_name)
        image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		#detecting chessboard corners
        ret, corners = cv2.findChessboardCorners(image_gray,(7,6),None)

        if ret == True:
            #points_3D.append(coordinates[0:corners.shape[0]])
            points_3D.append(coordinates)
            corners_new = cv2.cornerSubPix(image_gray,corners,(11,11),(-1,-1),criteria)
            points_2D.append(corners_new)
			#drawing chessboard corners
            img = cv2.drawChessboardCorners(image, (7,6), corners_new,ret)
            cv2.imshow('image',image)

    #save the 2D points i.e, image points(image coordinates)
    imgpt = open("2D_imagepoints.txt","w")
    for pt in points_2D[0]:
        for i in pt[0]:
            imgpt.write("%f " %i)
        imgpt.write("\n")
    imgpt.close()
	
	
	    #save the 3D points i.e, object points(world coordinates)
    objpt = open("3D_objectpoints.txt","w")
    for pt in points_3D[0]:
        for i in pt:
            objpt.write("\t%f" %i)
        objpt.write("\n")
    objpt.close()

    key = cv2.waitKey()
    if key == 27:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
