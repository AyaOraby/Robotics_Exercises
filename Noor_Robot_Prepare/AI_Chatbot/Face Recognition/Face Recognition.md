
# Face Detection Project

This project is a Python-based face detection application that uses OpenCV for detecting faces in images. It leverages machine learning and computer vision techniques to detect faces from input images.

## Prerequisites

Ensure that you have the following installed:

- **Python 3.x** (preferably Python 3.7 or higher)
- **pip** (Python's package installer)

## Installation

### 1. Clone the Repository (if applicable)

Clone the project repository to your local machine (skip this step if you already have the files).

```bash
git clone <repository_url>
cd facedetection-master
```

### 2. Install Required Packages

Use the following command to install the required Python packages listed in `requirements.txt`:

```bash
py -m pip install -r requirements.txt
```

This will install the necessary dependencies such as `numpy` and `opencv-python`.

### 3. Add Python to PATH (if needed)

If you encounter issues running Python scripts, ensure that Python is added to your system’s PATH. Here’s how you can do that:

1. Go to **System Properties** → **Environment Variables**.
2. Under **System variables**, find **Path**, then click **Edit**.
3. Add the Python installation directory and its `Scripts` folder:
   - Example: `C:\Users\<YourName>\AppData\Local\Programs\Python\Python3x\`
   - Example: `C:\Users\<YourName>\AppData\Local\Programs\Python\Python3x\Scripts\`

### 4. Check Python Installation

Open a new terminal or command prompt and check the Python installation:

```bash
python --version
```

or

```bash
py --version
```

You should see a Python version printed out (e.g., `Python 3.x.x`).

## Running the Face Detection Script

Once all dependencies are installed, you can run the `detect_face_image.py` script to detect faces in an image.

1. Ensure you are in the correct directory where `detect_face_image.py` is located.

2. Run the following command to detect faces in your image:

```bash
python detect_face_image.py
```

or, if `python` is not recognized:

```bash
py detect_face_image.py
```

This script will process the image and display the detected faces using OpenCV.

### Input Image

You can modify the script to point to a different image path or update the script to accept image paths dynamically.

## Troubleshooting

- If Python is not recognized, ensure you’ve added it to your system's PATH.
- If you encounter any issues with OpenCV, ensure you have the correct version installed (`opencv-python==4.11.0.86`).

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
