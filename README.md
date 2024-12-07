# VIVA ΛA VIΔA
# FUTURE-ENGINEERS-2025 
> ## HELLENIC TEAM
![LOGO_450](https://github.com/user-attachments/assets/4a292dbd-1131-4ac2-b4f5-d45ed665847f)


## TEAM MEMBERS:
### Nikol Vasilopoulou (user: Quart0xe)
### Panagiotis (user: )

# RESEARCH / EXPERIMENTAL PHASE
[Original module photos](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Robot%20Photos/Original%20module)

### Creating [red/green colour detection](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Code%20files/Python%20files/R_G_cannyoutlineWORKING.py)
Using opencv we created a program that detects red and green objects and outputs what the computer sees in the prossess.
To do this, firstly we used colour dropping to <ins> find approximate HSV values  of obstacles and edit the ranges in our code </ins>

> ### Red:

![image](https://github.com/user-attachments/assets/e6422d6a-328f-4d72-94b2-b9dd87e88be7)

> ### Green:

![image](https://github.com/user-attachments/assets/ffb01507-7a3f-49a9-9f3a-950e10f9cfc4)

### [Colour picker used](https://pinetools.com/image-color-picker)

Using the data, we created a range where an object is detected as red or green. The full range that can be detected is min:0 , max:255
```python
    red_lower = np.array([0, 120, 70])
    red_upper = np.array([10, 255, 255])
```
```python
    green_lower = np.array([45, 160, 0])
    green_upper = np.array([100, 255, 150])
```
Then, we applied a HSV mask to separate the objects of each colour from the rest of the feed and used imgcanny to find the corners of the objects detected, using that to add an outline according to their colour (red/green)
Mask:
```python
mask_red1 = cv2.inRange(imgHSV, red_lower, red_upper)
```
Outline:
```python
    img_contours = img.copy()
    for contour in contours_red:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red rectangle
```
Same for green.
> ### Green:

![image](https://github.com/user-attachments/assets/2dd7592b-8040-411a-b24d-54446ca6187f)
> ### Red:

![image](https://github.com/user-attachments/assets/cb52d939-d804-4d5f-a009-c82977a3d9f6)

Notice how it even detects the pile of red objects on the floor **~3m away**

### Next, we created the [Middle_Lane_Canny](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Code%20files/Python%20files/Middle_Lane_Canny_WORKING.py)
We applied the same code from the previous program, this time switching green and red with white and black in order to diffrentiate the (white) floor from the (black) walls.
From this process, instead of outlining the walls found, we used the line they formed from edge detection on each side and <ins> calculated the middle point between the two to create a path for the robot to follow </ins>
[See the video showing the output here]
