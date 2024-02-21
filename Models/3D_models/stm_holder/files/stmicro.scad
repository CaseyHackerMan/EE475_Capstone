pcb_width = 67.5;
pcb_length = 99;
pcb_height = 2.0;

header_width = 8.0;
header_length = pcb_length;
header_height = 25.0;
header_mount = 2.0;
header_horiz = 50.5;

button_side = 6;
button_base = 4;
button_height = 12;
button_bottom_space = 28;
button_horiz = 18;

mini_usb_width = 8;
mini_usb_height = 4;
mini_usb_depth = 8;

micro_usb_width = 8;
micro_usb_height = 4;
micro_usb_depth = 5;
micro_usb_right = 29.5;

usb_spacing = 3.0;

headphone_width = 8;
headphone_height = 13;
headphone_depth = 15;
headphone_left = 15;
headphone_jack_side = 8;
headphone_jack_out = 10;

usb_room = 10;

header = [header_width, header_length, header_height];
pcb = [pcb_width, pcb_length, pcb_height];

module header() {
	translate([0,0,-header_mount]) cube(header, center=true);
}
module pcb() {
	cube(pcb, center=true);
}
module button() {
	union() {
		translate([0,0,button_base/2]) cube([button_side, button_side, button_base], center=true);
		cylinder(r=button_side/2, h=button_height);
	}
}
module miniusb() {
	translate([-mini_usb_width/2,-mini_usb_depth/2,0]) union() {
		cube([mini_usb_width, mini_usb_depth, mini_usb_height]);
		translate([-usb_spacing/2,mini_usb_depth-0.1,-usb_spacing/2]) 
		cube([mini_usb_width + usb_spacing, usb_room, mini_usb_height + usb_spacing]);
	}
}
module microusb() {
	translate([-micro_usb_width/2,0,0]) union() {
		cube([micro_usb_width, micro_usb_depth, micro_usb_height]);
		translate([-usb_spacing/2,-usb_room+0.1,-usb_spacing/2]) 
		cube([micro_usb_width + usb_spacing, usb_room, micro_usb_height + usb_spacing]);
	}
}
module headphone() {
	union() {
		cube([headphone_width, headphone_depth, headphone_height]);
		translate([headphone_jack_side/2,0,headphone_height/2]) {
			rotate([90,0,0]) translate([0,headphone_jack_side/4,-0.1]){
				cylinder(r=headphone_jack_side/2,h=headphone_jack_out);
				translate([-headphone_jack_side/2,-headphone_jack_side/2,0])
				cube([headphone_jack_side, headphone_jack_side/2, headphone_jack_out]);
				translate([0,-headphone_jack_side/2,0])
				cylinder(r=headphone_jack_side/2,h=headphone_jack_out);
			}
		}
	}
}

module board() {
	union() {
		translate([-header_horiz/2,0,0]) header();
		translate([header_horiz/2,0,0]) header();
		translate([0,-pcb_length/2 + button_bottom_space,0]) {
			translate([-button_horiz/2,0,0]) button();
			translate([button_horiz/2,0,0]) button();
		}
		translate([0,pcb_length/2 - mini_usb_depth, pcb_height/2]) miniusb();
		translate([pcb_width/2-micro_usb_right,-pcb_length/2, pcb_height/2]) microusb();
		translate([-pcb_width/2 + headphone_left,-pcb_length/2,pcb_height/2]) headphone();
		pcb();
	}
}

//MAIN
bottom_h = 12;
ltol = 6;
wtol = 6;
lt = 0;
wt = 0;
deep = 1;
bw=pcb[0]+wtol;
bl=pcb[1]+ltol;
bwi=pcb[0]+lt;
bli=pcb[1]+wt;
module bottom() {
	difference() {
		translate([0,0,deep-bottom_h/2]) cube([bw, bl, bottom_h], center=true);
		board();
		translate([-bwi/2,-bli/2,0]) cube([bwi, bli, 50]);
		translate([0,0,deep-bottom_h/2+0.7]) {
			cube([20,pcb[1],bottom_h], center=true);
			translate([0,-ltol/4,0])
			cube([38,pcb[1]-ltol/2,bottom_h], center=true);
		}
	}
}

tltol = 9.5;
twtol = 9.5;
tw = pcb[0]+twtol;
tl = pcb[1]+tltol;
top_h = 15;
th = top_h + bottom_h - deep;
bdeep = 0;
ttol = 0.7;
tgap = 5;
module top() {
	difference() {
		translate([-tw/2,-tl/2,deep-bottom_h]) cube([tw, tl, th]);
		board();
		translate([0,0,top_h/2+bdeep]) {
			cube([20,pcb[1],top_h+2], center=true);
			cube([39,pcb[1],top_h+2], center=true);
		}
		translate([0,0,-bottom_h/2+deep-20/2])
		cube([bw+ttol,bl+ttol,bottom_h+20.1],center=true);
		translate([(header_horiz+header_width+tgap-0.1)/2,0,top_h/2])
		cube([tgap,pcb[1],top_h-2], center=true);
		translate([-(header_horiz+header_width+tgap-0.1)/2,0,top_h/2])
		cube([tgap,pcb[1],top_h-2], center=true);
	}
}

//board();
difference() {
	//top();
	bottom();
}