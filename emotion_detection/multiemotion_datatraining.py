import numpy as np
import cv2
import matplotlib.pyplot as plt
from sklearn.svm import SVC, LinearSVC
from sklearn.cross_validation import cross_val_score, KFold
from scipy.stats import sem
from sklearn import metrics
from PIL import Image
import glob
from scipy.ndimage import zoom
# from termcolor import colored
from matplotlib.patches import Rectangle


######################################################################
#                        data processing                             #
######################################################################

svc_1 = SVC(kernel='linear', decision_function_shape='ovo')


def evaluate_cross_validation(clf, X, y, K):
    # create a k-fold cross validation iterator
    cv = KFold(len(y), K, shuffle=True, random_state=0)
    # by default the score used is the one returned by score method of the estimator (accuracy)
    scores = cross_val_score(clf, X, y, cv=cv)
    print clf
    print (scores)
    print ("Mean score: {0:.3f} (+/-{1:.3f})".format(
        np.mean(scores), sem(scores)))


# training and evaluting
def train_and_evaluate(clf, X_train, X_test, y_train, y_test):

    clf.fit(X_train, y_train)

    print ("Accuracy on training set:")
    print (clf.score(X_train, y_train))
    print ("Accuracy on testing set:")
    print (clf.score(X_test, y_test))

    y_pred = clf.predict(X_test)

    print ("Classification Report:")
    print (metrics.classification_report(y_test, y_pred))
    print ("Confusion Matrix:")
    print (metrics.confusion_matrix(y_test, y_pred))

######################################################################
#                             opencv                                 #
######################################################################

# face detection
def detect_face(frame):
    cascPath = "haarcascade_frontalface_default.xml"
    faceCascade = cv2.CascadeClassifier(cascPath)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detected_faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=6,
            minSize=(100, 100),
            flags = 0
            #flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )
    return gray, detected_faces


# change face's size
def extract_face_features(gray, detected_face, offset_coefficients):
    (x, y, w, h) = detected_face
    horizontal_offset = offset_coefficients[0] * w
    vertical_offset = offset_coefficients[1] * h
    extracted_face = gray[y+vertical_offset:y+h,
                      x+horizontal_offset:x-horizontal_offset+w]
    new_extracted_face = zoom(extracted_face, (64. / extracted_face.shape[0],
                                           64. / extracted_face.shape[1]))
    # new_extracted_face = new_extracted_face.astype(float)
    # new_extracted_face /= float(new_extracted_face.max())
    return new_extracted_face


# use classifier to classify the face
def predict_face(extracted_face):

    return svc_1.predict(extracted_face.ravel())

def test_recognition(c1, c2):
    subplot(121)
    extracted_face1 = extract_face_features(gray1, face1[0], (c1, c2))
    imshow(extracted_face1, cmap='gray')
    print(predict_face(extracted_face1))
    subplot(122)
    extracted_face2 = extract_face_features(gray2, face2[0], (c1, c2))
    imshow(extracted_face2, cmap='gray')
    print(predict_face(extracted_face2))

def make_map(facefile):
    c1_range = np.linspace(0, 0.35)
    c2_range = np.linspace(0, 0.3)
    result_matrix = (float("inf")-float("inf")) * np.zeros_like(c1_range * c2_range[:, np.newaxis]) # set the matrix equals zero
    gray, detected_faces = detect_face(cv2.imread(facefile))
    for face in detected_faces[:1]:
        for ind1, c1 in enumerate(c1_range):
            for ind2, c2 in enumerate(c2_range):
                extracted_face = extract_face_features(gray, face, (c1, c2))
                result_matrix[ind1, ind2] = predict_face(extracted_face)
    return (c1_range, c2_range, result_matrix)

def main():

    # image preprocessing
    image_list = []
    image_data = []
    image_label = []
    image_target = []
    immax = []
    length = []
    di = 0
    ha = 0
    sa = 0
    su = 0
    # for filename in glob.glob('jaffe/*.tiff'):
    for filename in glob.glob('data/image/*.png'):

        # label the dataset
        im_label = filename[29]
        # if filename[9:11] == 'HA':
        #     im_label = 1
        # elif filename[9:11]== 'SA':
        #     im_label = 2
        # elif filename[9:11]== 'SU':
        #     im_label = 3
        # elif filename[9:11]== 'AN':
        #     im_label = 4
        # elif filename[9:11]== 'DI':
        #     im_label = 5
        # elif filename[9:11]== 'FE':
        #     im_label = 6
        # elif filename[9:11]== 'NE':
        #     im_label = 7
        if im_label == '3':
            di = di+1
        elif im_label == '5':
            ha = ha +1
        elif im_label == '6':
            sa = sa+1
        elif im_label == '7':
            su = su+1
        input_face = cv2.imread(filename)
        #im = Image.open(filename)
        gray, detected_faces = detect_face(input_face)
        # exrtact face
        for face in detected_faces:
            (x, y, w, h) = face
            if w > 100:
                face = extract_face_features(gray, face, (0.03, 0.05))
                #im_mat = np.array(face)
                # Use SIFT to xract features
                sift = cv2.SIFT()
                #sift = cv2.xfeatures2d.SIFT_create()
                # kp, des = sift.detectAndCompute(extracted_face, None)
                step_size = 3
                kp = [cv2.KeyPoint(x, y, step_size) for y in range(0, face.shape[0], step_size)
                                                    for x in range(0, face.shape[1], step_size)]
                kp, dense = sift.compute(face, kp)
        # dense=cv2.FeatureDetector_create("Dense")
        # kp=dense.detect(extracted_face)
        # kp,des=sift.compute(extracted_face,kp)

        #im_mat = np.array(im)
        im_mat = dense
        # im_mat = np.array(extracted_face)
        im_data = im_mat.flatten()
        image_list.append(input_face)
        image_data.append(im_data)
        image_label.append(im_label)
    # machine learning training and prediction

    train_data = image_data[:700]
    train_labels = image_label[:700]

    test_data = image_data[700:]
    test_labels = image_label[700:]

    evaluate_cross_validation(svc_1, train_data, train_labels, 5)
    train_and_evaluate(svc_1, train_data, test_data, train_labels, test_labels)

    #test2
    # input_face1 = cv2.imread('test.tiff')
    # input_face2 = cv2.imread('face_sad.png')
    # gray1, detected_faces1 = detect_face(input_face1)
    #     # exrtact face
    # for face in detected_faces1:
    #     (x, y, w, h) = face
    #     if w > 100:
    #         face1 = extract_face_features(gray1, face, (0.03, 0.05))
    #         #im_mat = np.array(face)
    # # Use SIFT to exract features
    #         sift = cv2.xfeatures2d.SIFT_create()
    #         # kp, des = sift.detectAndCompute(extracted_face, None)
    #         step_size = 3
    #         kp1 = [cv2.KeyPoint(x, y, step_size) for y in range(0, face1.shape[0], step_size)
    #                                             for x in range(0, face1.shape[1], step_size)]
    #         kp1, dense1 = sift.compute(face1, kp1)
    #         data1 = dense1.flatten()

    # gray2, detected_faces2 = detect_face(input_face2)
    #     # exrtact face
    # for face in detected_faces2:
    #     (x, y, w, h) = face
    #     if w > 100:
    #         face2 = extract_face_features(gray2, face, (0.03, 0.05))
    #         #im_mat = np.array(face)
    # # Use SIFT to exract features
    #         sift = cv2.xfeatures2d.SIFT_create()
    #         # kp, des = sift.detectAndCompute(extracted_face, None)
    #         step_size = 3
    #         kp2 = [cv2.KeyPoint(x, y, step_size) for y in range(0, face2.shape[0], step_size)
    #                                             for x in range(0, face2.shape[1], step_size)]
    #         kp2, dense2 = sift.compute(face2, kp2)
    #         data2 = dense2.flatten()

    # pre1 = svc_1.predict(data1)
    # pre2 = svc_1.predict(data2)
    # print pre1
    # print pre2

    ##################################################################
    #                             opencv                             #
    ##################################################################

    cascPath = "haarcascade_frontalface_default.xml"
    faceCascade = cv2.CascadeClassifier(cascPath)

    video_capture = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = video_capture.read()
        # detect faces
        gray, detected_faces = detect_face(frame)

        face_index = 0

        # predict output
        for face in detected_faces:
            (x, y, w, h) = face
            if w > 100:
                # draw rectangle around face
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # extract features
                face = extract_face_features(gray, face, (0.03, 0.05))
                sift = cv2.SIFT()
                step_size = 3
                kp = [cv2.KeyPoint(x, y, step_size) for y in range(0, face.shape[0], step_size)
                                                    for x in range(0, face.shape[1], step_size)]
                kp, dense = sift.compute(face, kp)
                data = dense.flatten()

                # predict smile
                prediction_result = svc_1.predict(data)
                #print prediction_result

                # draw extracted face in the top right corner
                #frame[face_index * 64: (face_index + 1) * 64, -65:-1, :] = cv2.cvtColor(extracted_face * 255, cv2.COLOR_GRAY2RGB)

                # annotate main image with a label
                if prediction_result == '5':
                    cv2.putText(frame, "Happy",(x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 155, 10)
                elif prediction_result == '6':
                    cv2.putText(frame, "Sad",(x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 155, 10)
                elif prediction_result == '7':
                    cv2.putText(frame, "Suprise",(x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 155, 10)
                # elif prediction_result == 4:
                #     cv2.putText(frame, "Angry",(x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 155, 10)
                elif prediction_result == '3':
                    cv2.putText(frame, "Disgust",(x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 155, 10)
                # elif prediction_result == 6:
                #     cv2.putText(frame, "Fear",(x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 155, 10)
                # elif prediction_result == 7:
                #     cv2.putText(frame, "Netural",(x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 155, 10)

                # increment counter
                face_index += 1


        # Display the resulting frame
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()




