function sysCall_init() 
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    
    for i=1,16,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)

    end
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    noDetectionDist=0.5 
    maxDetectionDist=0.2
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,-1.8, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.8,-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=5
    pioneer=sim.getObjectHandle("Pioneer_p3dx")
    beacon=sim.getObjectHandle("beacon")
    middle=sim.getObjectHandle("middle")
    positionp=sim.getObjectPosition(pioneer,-1)
    positionb=sim.getObjectPosition(beacon,-1)
    cam_pioneer=sim.getObjectHandle("cam_pioneer")
    cam_beacon=sim.getObjectHandle("cam_beacon")
    detection=sim.checkVisionSensor(cam_pioneer, beacon)
    timer1=50.0
    timer2=50.0
    counter1=0
    counter2=0
    time=sim.getSimulationTime()
 
--Basic variables, entry data for sensors-- 
end




function sysCall_actuation() 

    for i=1,16,1 do
        
        res,dist=sim.readProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else
            detect[i]=0
        end
    end
     
    vLeft=v0
    vRight=v0
   
 

    
    for i=1,16,1 do
    distance_to_beacon=sim.checkDistance(sim.getObjectHandle("Pioneer_p3dx"),sim.getObjectHandle("beacon"),0.51)
    distance_to_the_middle_of_the_room=sim.checkDistance(sim.getObjectHandle("Pioneer_p3dx"),middle,0.15)
    vLeft=vLeft+braitenbergL[i]*detect[i]
    vRight=vRight+braitenbergR[i]*detect[i]
    if(detect[i]==0)then
    vLeft=vLeft+braitenbergL[i]*detect[i]+0.1*math.random(-3,3)
    vRight=vRight+braitenbergR[i]*detect[i]+0.1*math.random(-3,3)
    elseif(detect[i]~=0)then
    vLeft=vLeft+braitenbergL[i]*detect[i]
    vRight=vRight+braitenbergR[i]*detect[i]
    end
    if(distance_to_beacon~=0 and counter1<1)then
      print("Beacon was found!")
      print(string.format("%.2f", timer1))
      vLeft=0
      vRight=0
      sim.setJointTargetVelocity(motorLeft,vLeft)
      sim.setJointTargetVelocity(motorRight,vRight)
      counter2=0
      if(timer1>0 and counter1<1)then
      timer1=timer1-sim.getSimulationTimeStep()
      else
      timer1=0
      if(timer1<0)then
      timer1=0
      vLeft=0
      vRight=0
      end
      if(timer1==0)then
      counter1=counter1+1
      vLeft=0
      vRight=0
      end
      if(timer1==0 and counter1>=0)then
      end
    end
    else
    if(distance_to_the_middle_of_the_room~=0 and counter2<1)then 
    
      print("I am approximately in the middle of the room.")
      print(string.format("%.2f", timer2))
      eulerAngles={7.5613e-05,2.5675e-03,3.141}
      --rotation=sim.getObjectOrientation(pioneer,-1)
      spin1=sim.setObjectOrientation(pioneer,-1,eulerAngles)
      vLeft=0
      vRight=0
      if(timer2>0 and counter2<1)then
      timer2=timer2-sim.getSimulationTimeStep()
      else
      timer2=0
      if(timer2<0)then
      timer2=0
      vLeft=0
      vRight=0
      end
      if(timer2==0)then
      counter2=counter2+1
      end
      if(timer2==0 and counter2>=1)then
      end
    end
      
    end
   
      
 
     --**************************--
     
 
    print(distance_to_beacon)
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
 end   
end 
if(timer2==0 and counter2>=1 and counter1>=1)then
sim.setJointTargetVelocity(motorLeft,0)
sim.setJointTargetVelocity(motorRight,0)
end
end