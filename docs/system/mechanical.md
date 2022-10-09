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
        height: 30rem
    }
    .hotspot{
        display: block;
        width: 20px;
        height: 20px;
        border-radius: 10px;
        border: none;
        background-color: blue;
        box-sizing: border-box;
        pointer-events: none;
    }
    .annotation{
        background-color: #888888;
        position: absolute;
        transform: translate(10px, 10px);
        border-radius: 10px;
        padding: 10px;
    }
    /* This keeps child nodes hidden while the element loads */
    :not(:defined) > * {
        display: none;
    }
</style>


<model-viewer alt="ESTHER 3D Model" src="../../assets/esther.glb" ar environment-image="" poster="" shadow-intensity="1" auto-rotate camera-controls touch-action="pan-y">
    <button class="hotspot" slot="hotspot-wheelchair" data-position="0 0 0" data-normal="0 0 1">
        <div class="annotation">Wheelchair Base</div>
    </button>
    <button class="hotspot" slot="hotspot-barrett" data-position="0 0.5 0.25" data-normal="0 0 1">
        <div class="annotation">Robot Arm</div>
    </button>
    <button class="hotspot" slot="hotspot-racket" data-position="0 0.75 0.5" data-normal="0 0 1">
        <div class="annotation">Tennis Racket</div>
    </button>
</model-viewer>


## Drivetrain
<img width="100%" src="../../assets/img/paper/chain_drive_2.png">
<img width="100%" src="../../assets/img/paper/chain_drive_3.png">

### Wheelchair
### Chain Drive

## Manipulator
### Barrett Arm
### End Effector
<img width="100%" src="../../assets/img/racket_clamp_stress.png">
<img width="100%" src="../../assets/img/racket_clamp_stress_animation.gif">
