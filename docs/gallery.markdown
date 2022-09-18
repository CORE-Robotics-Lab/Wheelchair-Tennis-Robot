---
layout: page
title: Gallery
permalink: /gallery/
---

<div id="carousel" class="carousel slide" data-bs-ride="carousel">
  <div class="carousel-indicators">
    <button type="button" data-bs-target="#carousel" data-bs-slide-to="0" class="active" aria-current="true" aria-label="Slide 1"></button>
    <button type="button" data-bs-target="#carousel" data-bs-slide-to="1" aria-label="Slide 2"></button>
    <!-- <button type="button" data-bs-target="#carousel" data-bs-slide-to="2" aria-label="Slide 3"></button> -->
  </div>
  <div class="carousel-inner">
    <div class="carousel-item active" data-bs-interval="2000">
      <img src="{{ 'assets/images/main_pic.png' | relative_url }}" class="d-block w-100" width="80%" alt="...">
      <div class="carousel-caption d-none d-md-block">
        <h5>ESTHER: mid swing pose</h5>
        <!-- <p>Some representative placeholder content for the first slide.</p> -->
      </div>
    </div>
    <div class="carousel-item" data-bs-interval="2000">
      <img src="{{ 'assets/images/complete_robot.jpg' | relative_url }}" class="d-block w-100" width="80%" alt="...">
      <div class="carousel-caption d-none d-md-block">
        <h5>ESTHER: arm at home position</h5>
        <!-- <p>Some representative placeholder content for the second slide.</p> -->
      </div>
    </div>
  </div>
  <button class="carousel-control-prev" type="button" data-bs-target="#carousel" data-bs-slide="prev">
    <span class="carousel-control-prev-icon" aria-hidden="true"></span>
    <span class="visually-hidden">Previous</span>
  </button>
  <button class="carousel-control-next" type="button" data-bs-target="#carousel" data-bs-slide="next">
    <span class="carousel-control-next-icon" aria-hidden="true"></span>
    <span class="visually-hidden">Next</span>
  </button>
</div>