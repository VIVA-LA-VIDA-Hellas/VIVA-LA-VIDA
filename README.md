# VIVA ΛA VIΔA
# FUTURE-ENGINEERS-2025 
> ## HELLENIC TEAM
![LOGO_450](https://github.com/user-attachments/assets/4a292dbd-1131-4ac2-b4f5-d45ed665847f)


## TEAM MEMBERS:
### Nikol Vasilopoulou (user: Quart0xe)
### Panagiotis (user: )

# RESEARCH / EXPERIMENTAL PHASE
[Original module photos](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/tree/main/Robot%20Photos/Original%20module)

###Creating [red/green colour detection](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Code%20files/Python%20files/R_G_cannyoutlineWORKING.py)
Using opencv we created a program that detects red and green objects and outputs what the computer sees in the prossess.
To do this, firstly we used colour dropping to find approximate HSV values of obstacles and edit the ranges in our code

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
Then, we applied a HSV mask to separate the objects of each colour from the rest of the feed
