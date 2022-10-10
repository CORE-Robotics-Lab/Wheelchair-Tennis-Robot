---
layout: page
title: Mechanical
permalink: /system/mechanical/
nav_order: 1
parent: System Design
---
# Mechanical
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }
- TOC
{:toc}


<script type="module" src="https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js"></script>
<!-- 3D model viewer style settings -->
<style>
    model-viewer {
        width: 100%;
        height: 40rem;
    }
    .hotspot{
        display: block;
        width: 18px;
        height: 18px;
        border-radius: 9px;
        border: none;
        background-color: #ffc800;
        box-sizing: border-box;
        pointer-events: none;
    }
    .annotation{
        background-color: #FFF;
        position: absolute;
        transform: translate(10px, 10px);
        border-radius: 10px;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.25);
        padding: 10px;
        font-size: 12px;
        font-weight: 700;
        max-width: 128px;
        width: max-content;
        height: max-content;
    }
    /* This keeps child nodes hidden while the element loads */
    :not(:defined) > * {
        display: none;
    }
</style>


<model-viewer alt="ESTHER 3D Model" src="../../assets/esther.glb" ar environment-image="" poster="" shadow-intensity="1" auto-rotate camera-controls touch-action="pan-y">
    <button class="hotspot" slot="hotspot-wheelchair" data-position="0.42 0.3 0.3" data-normal="0 0 1">
        <div class="annotation">Wheelchair Base</div>
    </button>
    <button class="hotspot" slot="hotspot-barrett" data-position="0.17 0.85 -0.16" data-normal="0 0 1">
        <div class="annotation">Robot Arm</div>
    </button>
    <button class="hotspot" slot="hotspot-end" data-position="0.2 1.05 -0.09" data-normal="0 0 1">
        <div class="annotation">End Effector</div>
    </button>
</model-viewer>

## Parts

| Item                                | Part Link                                                                                                                                                                                                                                                                  | Comments                              |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------- |
| Nema 23 1:10 gearbox (1x)           | [https://www.amazon.com/dp/B097SCZT59/](https://www.amazon.com/dp/B097SCZT59/)                                                                                                                                                                                             |                                       |
| T-Slot rail (82cm)                  | [https://www.mcmaster.com/47065T109/](https://www.mcmaster.com/47065T109/)                                                                                                                                                                                                 |                                       |
| T-Slot corner bracket (4x)          | [https://www.mcmaster.com/3136N172/](https://www.mcmaster.com/3136N172/)                                                                                                                                                                                                   |                                       |
| Motor enclosure (2x)                | [hardware/mechanical/wheelchair/motor_enclosure.stl](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_enclosure.stl)                                                                                            | 3D print out of PLA                   |
| Motor enclosure plate (2x)          | [hardware/mechanical/wheelchair/motor_enclosure_plate.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_enclosure_plate.DXF)                                                                                | Water jet out of thin metal           |
| Nema23 to Nema34 adapter plate (2x) | [hardware/mechanical/wheelchair/motor_nema23_nema34_plate.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_nema23_nema34_plate.DXF)                                                                        | Water jet out of thin metal           |
| Clamping shaft coupler (2x)         | [https://www.mcmaster.com/2469K74/](https://www.mcmaster.com/2469K74/)                                                                                                                                                                                                     |                                       |
| Keyed rotary shaft (1x)             | [https://www.mcmaster.com/1439K26/](https://www.mcmaster.com/1439K26/)                                                                                                                                                                                                     | Cut into two 10cm lengths             |
| Machine key (4x)                    | [https://www.mcmaster.com/98870A733/](https://www.mcmaster.com/98870A733/)                                                                                                                                                                                                 |                                       |
| Threaded hex standoffs (8x)         | [https://www.mcmaster.com/94868A767/](https://www.mcmaster.com/94868A767/)                                                                                                                                                                                                 |                                       |
| Motor mount plate left (1x)         | [hardware/mechanical/wheelchair/motor_mount_plate_left.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_mount_plate_left.DXF)                                                                              | Water jet out of metal sheet          |
| Motor mount plate right (1x)        | [hardware/mechanical/wheelchair/motor_mount_plate_right.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_mount_plate_right.DXF)                                                                            | Water jet out of metal sheet          |
| Taper-lock bushing (2x)             | [https://www.mcmaster.com/3753N111/](https://www.mcmaster.com/3753N111/)                                                                                                                                                                                                   |                                       |
| Taper-lock sprocket (2x)            | [https://www.mcmaster.com/57095K421/](https://www.mcmaster.com/57095K421/)                                                                                                                                                                                                 |                                       |
| Axle metal wedge left (1x)          | [https://www.mcmaster.com/6620K78/](https://www.mcmaster.com/6620K78/) [hardware/mechanical/wheelchair/axle_metal_wedge_left.SLDPRT](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/axle_metal_wedge_left.SLDPRT)   | Fabricate model out of metal block    |
| Axle metal wedge right (1x)         | [https://www.mcmaster.com/6620K78/](https://www.mcmaster.com/6620K78/) [hardware/mechanical/wheelchair/axle_metal_wedge_right.SLDPRT](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/axle_metal_wedge_right.SLDPRT) | Fabricate model out of metal block    |
| Sprocket (2x)                       | [https://www.mcmaster.com/6836N131/](https://www.mcmaster.com/6836N131/) [hardware/mechanical/wheelchair/motor_sprocket_plate.SLDPRT](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_sprocket_plate.SLDPRT)   | Drill specified holes out of sprocket |
| Roller chain (2x)                   | [https://www.mcmaster.com/6027K73/](https://www.mcmaster.com/6027K73/)                                                                                                                                                                                                     |                                       |
| Roller chain tensioner (2x)         | [https://www.mcmaster.com/6233K88/](https://www.mcmaster.com/6233K88/)                                                                                                                                                                                                     |                                       |
| Tensioner mount plate (2x)          | [https://www.mcmaster.com/4473T42/](https://www.mcmaster.com/4473T42/)                                                                                                                                                                                                     | Water jet out of metal sheet          |
| Tensioner mount standoff (2x)       | [hardware/mechanical/wheelchair/tensioner_mount_standoff.stl](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/tensioner_mount_standoff.stl)                                                                          | 3D print out of PLA                   |

## Drivetrain
We designed a chain-drive system to deliver power from the motors to the wheels. Power from the battery is delivered through an ODrive motor controller to two ODrive D6374 motors. The motors for both wheels are coupled to a 1:10 ratio speed-reducer planetary gearbox. The gearbox output shaft is coupled with the wheel through a chain and sprocket system that provides an additional 1:2 speed reduction to give a total reduction of 1:20. With the motors rotating at maximum speed, the wheelchair can achieve linear velocities of up to 10 m/s and in-place angular yaw velocity of up to 20 rad/s. A chain drive system has higher durability, torque capacity, increased tolerances, and a simpler design versus belt drive or friction drive systems.\

### Wheelchair
### Chain Drive
<img width="49%" src="../../assets/img/paper/chain_drive_2.png">
<img width="50%" src="../../assets/img/paper/chain_drive_3.png">

## Manipulator
### Barrett Arm
### End Effector
<img width="100%" src="../../assets/img/racket_clamp_stress.png">
<img width="100%" src="../../assets/img/racket_clamp_stress_animation.gif">


---

Now check out the [electrical](https://core-robotics-lab.github.io/Wheelchair-Tennis-Robot/system/electrical/) page to see how to setup the electrycal system. 
