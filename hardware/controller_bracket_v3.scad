difference() {
    import("controller_bracket_v2.stl");
    translate([80, 68, -2]) cube([30, 35, 20]);
    translate([10, 22, -10]) cube([25, 40, 20]);
    translate([80, 108, -2]) cube([30, 20, 20]);
    translate([-20, 15, 6]) button_punch();
    translate([-20, 30, 6]) button_punch();
    translate([0, 5, 6]) rotate([0,0,-90]) button_punch();

    translate([20,15,12]) rotate([90,0,0]) #cylinder($fn = 64, d = 4, h = 30);
}

module button_punch(clearance = 0) {
    // translate([0,0,50])
     dia = 4;
     translate([0,dia/2,dia/2])
     rotate([0,0,90])
	  for (j=[0, 1])  {
	       translate([0, 0, 9 * j])
	       for (i=[0, 1])  {
		    translate([6 * i, 0, 0])
			 rotate(a = [90,0,])
			 cylinder($fn = 64,
				  h = 15 + 2 * clearance,
				  d = dia + clearance/2);
	       }
	  }
}
