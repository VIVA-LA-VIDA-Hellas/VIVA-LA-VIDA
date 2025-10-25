# VIVA ΛA VIΔA
# FUTURE-ENGINEERS-2025 
> ## HELLENIC TEAM
![LOGO_450](https://github.com/user-attachments/assets/4a292dbd-1131-4ac2-b4f5-d45ed665847f)


## TEAM MEMBERS:
### Nikol Vasilopoulou (user: Quart0xe)
### Panagiotis Mourmouris (user: Panos1431)
### Thanos Karatsis (user: thkaratsis)


# INDEX
### *Use the index for an easy navigation of our github or, continue bellow to view our creation process, as well as an overview on everything about the robot.*

## 1. Mobility management
####        1.1 Motor
####        1.2 Ackerman
####        1.3 Chassis
####        1.4 Servo
####        1.5 Servo and motor 3d Bases
####        1.6 Photos

## 2. Power Management
####        2.1 Batteries
####        2.2 [PCB circuit designs and schematics](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Schematics/PCB)
####        2.3 Wiring and wiring managment 

## 3. Sensors
####        2.1 Distance sensors
####        2.2 Accelerometre / Gyro
####        2.3 Buttons
####        2.4 Camera
####        2.5 Colour sensor
####        2.6 3d sensor bases

## 4. Obstacle Management 
####        4.0 Flowchart
####        4.1 Wall sensing
####        4.2 Direction recognition
####        4.3 Red/Green differentiation
####        4.4 Turning and turn count tracking
####        4.5 Unparking / Parking

## 5. Pictures 
####        5.1 [Team photos](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Team%20photos)
####        5.2 [Vehicle photographs](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Robot%20Photos)

## 6. Performance videos
####        6.1 [Youtube channel link](https://www.youtube.com/@VIVALAVIDAFutureEnginners)
####        6.2 [All video links](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Videos)

## 7. Daily entries
####        7.1 [Logs](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Training%20meetings%20log)

## 8. 3D Prints
####        8.1 [3D schematics](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/3d%20designs/3d%20schematics)
####        8.2 [Photos of 3D printed parts](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/3d%20designs/3d%20printed%20part%20photos)

## 9. Manuals
####        9.1 Instruction manual
####        9.2 Build details

## 10. Extra material
####        10.1 [Team logo](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Team%20-%20Extras/Logo)
####        10.2 [Team name - VIVA ΛΑ VIΔΑ](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Team%20-%20Extras/Name)
####        10.3 [Team shirts](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Team%20-%20Extras/Shirts)


    


# RESEARCH / EXPERIMENTAL PHASE
[Original module photos](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Robot%20Photos/Original%20module)

### Creating [red/green colour detection](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Code%20files/Python%20files/R_G_cannyoutlineWORKING.py)
Using python and opencv we created a program that detects red and green objects and outputs what the computer sees in the prossess.
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

Notice how it even detects the pile of red objects on the floor **~3m away**, We want to turn based on the closest object to the robot at any given time.
To detect that, we add limits to the size we consider an object 
> We do this so that it doesnt start turning too early or if it sees a spec of green/red in the background), its basically a filtering process that cuts down on most of the >noise.

For trials we set the minimum requirement in size (area) for a detected colour mass to be considered an object, as **500 pixels**
```python
def get_largest_contour(mask, min_area=500)
```

Next, we made it choose the largest object visible to focus on first
```python
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area: return None
    x, y, w, h = cv2.boundingRect(largest)
    return largest, (x, y), (x + w, y + h), w*h
```

### Updating our colour picking method
The issue with this code was that it needed calibrating in different lighting conditions, so we made out own [colour picker](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Code%20files/Python%20files/New%20files%20September%2B/Colour_picker.py) that worked more accurately with the hsv values our camera detected. With this the obstacle detection became much more time efficient but also accurate.

Colour picker sample
```python
def mouse_callback(event, x, y, flags, param):
    global frame
    if event == cv2.EVENT_LBUTTONDOWN and frame is not None:
        bgr = frame[y, x]
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        print(f"Clicked at ({x},{y}) | BGR: {bgr} | HSV: {hsv}")

cv2.namedWindow("Live Feed")
cv2.setMouseCallback("Live Feed", mouse_callback)
```
We use input from our camera to our computer as a live feed, clicking on any pixel to get the HSV values the camera detects straight to our terminal.
We can then imput them straight into our colour detection limits, (red_lower, red_upper, green_lower, green_upper)

### Using what we detected to avoid the obstacles
Now that we can succesfully detect the red and green obstacles, we decided to implement a simple system for the robot to use in order to turn. 
It uses the Objects detected as a reference point and tries to rotate away from them, untill they are outside of the visible area (in other words, when they are far away enough for the robot to not collide with them).
Reminder, the robot must move **right** if it detects **red** and on the flip side, **left** when it detects **green**
> In other words, if we draw a line on the right side of the "turn right" block (red) and the left side of the "turn left" block (green), by moving toward the side it is placed on and effectively moving the entire obstacle out of our view, we manage to pass it.

So for **Green** (Turn left):

<img width="576" height="360" alt="green pillar" src="https://github.com/user-attachments/assets/0db5fb66-a389-43f2-9f20-6747467d4814" />

And for **Right** (Turn right):

<img width="576" height="360" alt="red pillar" src="https://github.com/user-attachments/assets/67523d32-0b6a-445d-900d-beb8b43237e5" />



### Next, we created the [Middle_Lane_Canny](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Code%20files/Python%20files/Middle_Lane_Canny_WORKING.py)
We applied the same code from the previous program, this time switching green and red with white and black in order to diffrentiate the (white) floor from the (black) walls.
From this process, instead of outlining the walls found, we used the line they formed from edge detection on each side and <ins> calculated the middle point between the two to create a path for the robot to follow </ins>
```python
for i in range(height):
        left_avg = left_edges[i]
        right_avg = right_edges[i]

        if left_avg is not None and right_avg is not None:
            # Calculate the midpoint and draw the path
            middle_line = (left_avg + right_avg) // 2

            # Draw the path line (white) with increased thickness
            cv2.line(edges_black, (middle_line, i), (middle_line, i), (255, 255, 255), 3)  # Increased thickness
```

[You can see the output we got here](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Videos/MIDDLE_LANE_VIDEO_COMPRESSED.mp4)

### Output image
![image](https://github.com/user-attachments/assets/417dd036-7add-4786-abe6-badace10178b)
### When we moved the inner square walls, <ins>the path to follow (middle lane) moved too</ins>
![image](https://github.com/user-attachments/assets/95e8c7f5-c216-4cda-be64-c463440ad920)

### Why we didn't end up using this code (Middle_Lane_Canny) for the first mission
Although this code worked exceptionally well when the robot was faced with long straight paths surrounded by contrasting walls, the same can't be said for turns.
When we tested it with a large camera connected to a laptop, it was facing the track from a very high up angle so it had a better view and could detect turns well.
But, when we tested multiple different cameras connected to a raspberry pi, including a fisheye camera, none of them has a scope wide enough for the current code to predict sharp turns, so the path ended up either guiding the robot straight into the wall, or, turning the opposite direction than expected.  

This wasnt working for us. The camera couldn't be as high up as it was originally when its placed on the robot -in the track- and it also needed the robot to go relatively slow so it can catch up with the camera feedback. This is fine for the second mission, where obstacle avoidance is a higher priority than speed, but for a quick first mission we had to find a new way to sense when and how much the car needs to turn.

### Recognising turns - Detecting Blue and Orange lines
## $${\color{blue}Blue}$$
## $${\color{orange}Orange}$$




