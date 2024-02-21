stm_pcb_thickness = 1.65;
stm_width_x = 97.1;
stm_width_y = 66.1;
stm_pin_length_down = 10.1;
stm_pin_length_up = 4.2;
stm_pin_dual_row_width_y = 5;
stm_pin_dual_row_width_x = 64.5;
stm_pin_dual_row_distance_from_border = 5;
stm_height_above_pcb_audio = 12.5;
stm_audio_width_x = 14.1;
stm_audio_width_y = 8.3;
stm_audio_pos_x = -(stm_width_x-stm_audio_width_x)/2;
stm_audio_pos_y = 14;
stm_audio_jack_diameter = 6;
stm_audio_jack_length = 3.9;
stm_audio_jack_pos_x = -(stm_width_x)/2 - stm_audio_jack_length/2;
stm_audio_jack_pos_y = 14.5;
stm_audio_jack_pos_z = 6.7 +stm_pcb_thickness;

stm_height_above_pcb_buttons = 9.8;
stm_height_above_pcb_jumpers = 8.5;
stm_height_overall = 3.6;




module stm32f4discovery(){
	union(){
		//PCB
		cube(size=[stm_width_x, stm_width_y, stm_pcb_thickness], center=true);
		//dual row pin headers
		translate([-stm_width_x/2,0,0]){
			translate([0,0,stm_pcb_thickness/2]){
				translate([0,stm_width_y / 2 - stm_pin_dual_row_width_y - stm_pin_dual_row_distance_from_border,0]) cube(size=[stm_pin_dual_row_width_x, stm_pin_dual_row_width_y, stm_pin_length_up]);
				translate([0, - stm_width_y / 2 + stm_pin_dual_row_distance_from_border,0]) cube(size=[stm_pin_dual_row_width_x, stm_pin_dual_row_width_y, stm_pin_length_up]);
			}
			translate([0,0,-stm_pcb_thickness/2-stm_pin_length_down]){
				translate([0,stm_width_y / 2 - stm_pin_dual_row_width_y - stm_pin_dual_row_distance_from_border,0]) cube(size=[stm_pin_dual_row_width_x, stm_pin_dual_row_width_y, stm_pin_length_down]);
				translate([0, - stm_width_y / 2 + stm_pin_dual_row_distance_from_border,0]) cube(size=[stm_pin_dual_row_width_x, stm_pin_dual_row_width_y, stm_pin_length_down]);
			}
		}
		//audio jack cubic part
		translate([stm_audio_pos_x,stm_audio_pos_y, (stm_height_above_pcb_audio+
stm_pcb_thickness)/2]) cube(size=[stm_audio_width_x, stm_audio_width_y, stm_height_above_pcb_audio],center=true);
		//audio jack round part
		translate([stm_audio_jack_pos_x,stm_audio_jack_pos_y,stm_audio_jack_pos_z]) {rotate(a=[0,90,0]) cylinder(h=stm_audio_jack_length, r=stm_audio_jack_diameter/2, center=true);}
	}
}

stm32f4discovery();