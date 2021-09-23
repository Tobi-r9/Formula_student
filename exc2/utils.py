import os
import cv2
import numpy as np

def load_data(path, labels, shape=(128,128)):
    """loads image data from path

    Args:
        path (string): path to directory with files
        labels (list): list of labels, labels have to be in filename of image
        shape (tuple, optional): image shape

    Returns:
        np.array: images, labels
    """
    images = []
    y = []
    data = [os.path.join(path, f) for f in os.listdir(path)]
    for image_path in data:
        img = cv2.imread(image_path)
        img = cv2.resize(img, shape)
        img = img/255
        images.append(img)

        for i, label in enumerate(labels):
            if label in image_path:
                y.append(i)
                break
    y = np.array(y)
    images = np.array(images)
    return images, y