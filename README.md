# ERIC_A (Erica Robot Inteligence Car - Automatic delivery)

## Project Team :  Deli-Bery

### Member : 

[김정한(Jeonghan Kim)](https://github.com/jeong-han-kim), [배가연(Gayeon Bae)](https://github.com/BaeGayeon), [박종찬(Jongchan Park)](https://github.com/coldbell8918), [이하연(Hayeon Lee)](https://github.com/quokkalover), [권기현(GIhyeon Gi)](https://github.com/raisewise0211), [강지훈(Jihun Kang)](https://github.com/jh4946)



---



#### Robot Hardware Developer : 

[강지훈(Jihun Kang)](https://github.com/jh4946), [이하연(Hayeon Lee)](https://github.com/quokkalover)

#### Robot Software Developer : 

[김정한(Jeonghan Kim)](https://github.com/jeong-han-kim), [박종찬(Jongchan Park)](https://github.com/coldbell8918), [이하연(Hayeon Lee)](https://github.com/quokkalover), [권기현(GIhyeon Gi)](https://github.com/raisewise0211)

#### Web Developer : 

[배가연(Gayeon Bae)](https://github.com/BaeGayeon)

---



## Abstract

It is an ERIC_A indoor delivery robot. It is a robot made for delivery of goods indoors. Basically, ERIC_A is equipped with an autonomous driving system, so it accurately identifies its current location and arrives at its destination. Also, the robot is equipped with YOLO artificial intelligence, so when you see a person on your path, it says, "The robot passes by."
After logging in through the web page, enter your name, item, and delivery address to proceed with the order. When the order is accepted, the order is accumulated in the database of the administrator page. The administrator can select from the ordered details and start shipping.
When the robot arrives at the administrator's store, the robot notifies it, and the manager puts goods into the robot through the administrator page. When the administrator presses the delivery start, the robot starts shipping to the delivery destination, completes the delivery, and returns to the start station.

Please refer to YouTube for more detailed understanding.

## workflow
<p>
<div class=pull-right>
<img width="523" alt="flow" border="0" src="https://user-images.githubusercontent.com/98142496/209927234-fd644c19-fefb-4b4a-8ae4-0bc33c15ecf4.png"> 
</div>

</p>

## Hardware system : 

### Design

<img src="https://user-images.githubusercontent.com/98142496/209920864-a059d868-698b-4810-b211-bdf6e99cc63f.jpg" width="360" height="360"/>    ![as](https://user-images.githubusercontent.com/98142496/209925051-a3b93e7e-aa9f-402b-b194-f23531c2aa6c.jpg)



### Architecture
![hardware Architecture](https://user-images.githubusercontent.com/98142496/209920998-c9386c03-3ca8-4c1e-8cb2-1d6f09950620.png)

## Software system :

#### web Architecture
![Software Architecture](https://user-images.githubusercontent.com/98142496/209925273-55ec6f5d-9286-4f32-b214-14abc07b7b17.jpg)

### Interface
                     user page                                administrator page
<img width="373" alt="접수" src="https://user-images.githubusercontent.com/98142496/209925531-5afb6480-846f-4bd0-a23a-06217aa2a99a.png">  <img width="373" alt="접수상세조회-접수지도착" src="https://user-images.githubusercontent.com/98142496/209925606-5ce08925-ab3e-4d4f-b674-4cf09bfc8e44.png">

If you are curious about the contents related to the web, click this link. [ERIC-A-web-backend-expressjs](https://github.com/CrashLab-2022/ERIC-A-web-backend-expressjs), [ERIC-A-web-frontend-nuxtjs](https://github.com/CrashLab-2022/ERIC-A-web-frontend-nuxtjs)

### robot System Architecture
<img src="https://user-images.githubusercontent.com/98142496/209921060-054379c7-239c-4820-9ed8-104d54dac2e0.png" width="80%"/>

## How to run
### install
```
git clone --recurse-submodules https://github.com/CrashLab-2022/ERIC_A.git
```
### Requirements
```
# (on eric_a_bringup/scritps) 
sudo ./build.sh
```

#### Launch
```
roslaunch eric_a_bringup eric_a_robot.launch
```

## Additional Resources
- [Youtube](https://www.youtube.com/)
- [ERICA'22 Presentaion ](https://docs.google.com/presentation/)
