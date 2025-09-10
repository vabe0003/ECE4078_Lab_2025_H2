import cv2, glob

CHECKERBOARD = (6, 9)
images = glob.glob("images/calib_*.png")

print(f"Found {len(images)} images")
for f in images:
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    print(f, "Corners found:", ret)

    if ret:
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow("Checkerboard", img)
        cv2.waitKey(300)  # show each image for 0.3s

cv2.destroyAllWindows()