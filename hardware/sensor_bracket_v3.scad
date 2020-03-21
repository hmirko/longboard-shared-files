$fn = 128;
eps = 0.02;

sensor_x_offset = 10;

// mounting on the axis
axis_screw_d = 5; // diameter
axis_screw_mb = 6; // margin front
axis_screw_ms = 5; // margin side
axis_screw_dl = 62; // distance longitudinal

// axis base mounting plate
axis_plate_h = 5;
axis_plate_l = 87;
axis_plate_w = 55;

// load cell sensor
sensor_d = 20;
sensor_h = 10;
sensor_pin_d = 2.5;
sensor_pin_h = 1.8;
sensor_screw_d = 2.7;
sensor_screw_b = 0.8;

m6_mutter_d = 9;
m6_mutter_h = 4;

bracket_l = sensor_d * 1.5;
bracket_h = sensor_h + 10;
bracket_h_max = 14;

bar_w = 13; // todo
bar_h = bracket_h;

side_bar_w = 3; // todo
side_bar_h = axis_plate_h;

wing_w = 20;
wing_h = 2;

difference() {
  union() {
    sensor_bracket(clearance = 0.4);
  }
  //translate([-1 + sensor_d,-1 + axis_plate_w/2,-1]) cube([10,100,100]);
}

module ft_sensor(clearance = 0, cutout = 0) {
  r = sensor_d / 2;
  screw_r = sensor_screw_d/2;
  loop_r = r - screw_r - sensor_screw_b;
  connector_h = axis_plate_w;
  connector_d = sensor_h * 0.6 + 2*clearance;
  screw_count = 3;
  union() {
    translate ([sensor_x_offset, 0, - clearance])
      cylinder (h=sensor_h * 1 + 2 * clearance, d = sensor_d + 2 * clearance);

    translate([sensor_x_offset,0,sensor_h*0.5]) rotate([90,0,0])
      cylinder(d = connector_d, h = connector_h);

    if (cutout) {
      for (i=[1:screw_count])  {
	rotation = 90;
	translate([loop_r*cos(i*(360/screw_count) + rotation) + sensor_x_offset,
		   loop_r*sin(i*(360/screw_count) + rotation),
		   0])
	  cylinder(r= screw_r + clearance, h = bracket_h + bracket_h_max + clearance);
      }
      translate([sensor_x_offset,0,- sensor_pin_h * 3 - eps])
	cylinder(h = sensor_pin_h*3, d = sensor_d + 2*clearance);

      translate([- connector_d/2 + sensor_x_offset,
		 -connector_h,
		 -sensor_h/2])
	cube([connector_d, connector_h, sensor_h]);
    }
  }
}

module sensor_bracket(clearance = 0) {
  union() {

    /* for(i = [0, 1]) { */
    /*   translate([0, */
    /* 		 i * axis_plate_w - (1 - i) * wing_w, */
    /* 		 bracket_h - wing_h]) */
    /*   difference() { */
    /* 	cube([bracket_l, wing_w, wing_h]); */
    /* 	translate([bracket_l/2, wing_w/2,-eps]) */
    /* 	cylinder(d = axis_screw_d, h = wing_h * 2); */
    /*   } */

    /*   } */
    difference() {
      translate([0,-side_bar_w,0])
	cube([bracket_l, axis_plate_w + 2* side_bar_w, bracket_h]);
      translate([(bracket_l - sensor_d) /2,
		 axis_plate_w /2,
		 -bracket_h * 0.01 + sensor_pin_h * 0.3])
	ft_sensor(cutout = true, clearance = clearance);

      oversize_factor = 2.5;
      translate([-eps,
		  -eps - (oversize_factor - 1)/2 * axis_plate_w,
		  -eps - axis_plate_h*1.3])
	scale([bracket_l + 2*eps,
	       oversize_factor * axis_plate_w + 2*eps,
	       axis_plate_h*2])
		difference() {
		  cube ([1,1,1]);
		  d = 0.4;
		  translate ([-1, 0.5, 0.8 + d/2])
		    rotate(a = 90, v = [0,1,0])
		    cylinder(d = d, h = 3);
		}
		}

    difference() {
      for(i = [0,1]) {

	downward_shift =  side_bar_h/2 - + clearance;
	translate([bracket_l,
		   axis_plate_w * i + (i-1) * side_bar_w,
		   + axis_plate_h  - downward_shift])
	  cube([ axis_plate_l, side_bar_w, bar_h + downward_shift]);

	bar_z_shift = axis_plate_h;
	slope_l = bracket_l;

	translate([bracket_l,
		   i * (axis_plate_w - bar_w) ,
		   bar_z_shift]) {
	  cube([axis_plate_l,
		bar_w,
		bar_h]);
	  translate([- slope_l,
		     0,
		     bar_h - bar_z_shift])
	    difference() {
	    cube([slope_l, bar_w, bar_z_shift]);

	    a = bar_z_shift;
	    b = slope_l;
	    c = sqrt(a*a + b*b);
	    slope_angle = asin(a/c);

	    translate([0, -bar_w, 0])
	      rotate(a = -slope_angle, v = [0,1,0])
	      cube([2 * slope_l, bar_w * 3, bar_z_shift * 2]);
	  }
	}
      }
      translate([bracket_l + axis_screw_d / 2,
		 0,
		 0]) {
	for(i = [-1, 1]) {
	  xpos = axis_screw_mb;
	  ypos = axis_plate_w/2
	    + i* axis_plate_w/2
	    + -i * (axis_screw_d / 2 + axis_screw_ms);

	  for(s = [0, 1]) {
	    translate([xpos + s * axis_screw_dl, ypos, 0])
	      union() {
	      z_top_bar = bracket_h + axis_plate_h + clearance;

	      cylinder(h = z_top_bar + eps,
		       d = axis_screw_d + clearance * 2);
	      translate([0, 0, z_top_bar - m6_mutter_h])
		cylinder(h = m6_mutter_h * 1.5,
			 d =  m6_mutter_d + clearance * 2,
			 $fn=6);
	    }
	  }
	}
      }
    }
  }
}
