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


## Drivetrain
We designed a chain-drive system to deliver power from the motors to the wheels. Power from the battery is delivered through an ODrive motor controller to two ODrive D6374 motors. The motors for both wheels are coupled to a 1:10 ratio speed-reducer planetary gearbox. The gearbox output shaft is coupled with the wheel through a chain and sprocket system that provides an additional 1:2 speed reduction to give a total reduction of 1:20. With the motors rotating at maximum speed, the wheelchair can achieve linear velocities of up to 10 m/s and in-place angular yaw velocity of up to 20 rad/s. A chain drive system has higher durability, torque capacity, increased tolerances, and a simpler design versus belt drive or friction drive systems.\

### Wheelchair
### Chain Drive
<img width="50%" src="../../assets/img/paper/chain_drive_2.png">
<img width="50%" src="../../assets/img/paper/chain_drive_3.png">

## Manipulator
### Barrett Arm
### End Effector
<img width="100%" src="../../assets/img/racket_clamp_stress.png">
<img width="100%" src="../../assets/img/racket_clamp_stress_animation.gif">


---

Now check out the [electrical](https://core-robotics-lab.github.io/Wheelchair-Tennis-Robot/system/electrical/) page to see how to setup the electrycal system. 
