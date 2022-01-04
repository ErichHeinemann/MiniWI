// At the end, there are a lot of disabled statements to print the needed pieces.
// I prefer to design the STL for a single piece and then add them in the Slicer as often I need them

// OpenSource GPL 

// E.Heinemann 2022



radPipe = 50; // 10cm
radRing = 6;


$fs = 0.15;
$fn = 50;

module roundedcube(size = [1, 1, 1], center = false, radius = 0.5, apply_to = "all") {
	// If single value, convert to [x, y, z] vector
	size = (size[0] == undef) ? [size, size, size] : size;

	translate_min = radius;
	translate_xmax = size[0] - radius;
	translate_ymax = size[1] - radius;
	translate_zmax = size[2] - radius;

	diameter = radius * 2;

	obj_translate = (center == false) ?
		[0, 0, 0] : [
			-(size[0] / 2),
			-(size[1] / 2),
			-(size[2] / 2)
		];

	translate(v = obj_translate) {
		hull() {
			for (translate_x = [translate_min, translate_xmax]) {
				x_at = (translate_x == translate_min) ? "min" : "max";
				for (translate_y = [translate_min, translate_ymax]) {
					y_at = (translate_y == translate_min) ? "min" : "max";
					for (translate_z = [translate_min, translate_zmax]) {
						z_at = (translate_z == translate_min) ? "min" : "max";

						translate(v = [translate_x, translate_y, translate_z])
						if (
							(apply_to == "all") ||
							(apply_to == "xmin" && x_at == "min") || (apply_to == "xmax" && x_at == "max") ||
							(apply_to == "ymin" && y_at == "min") || (apply_to == "ymax" && y_at == "max") ||
							(apply_to == "zmin" && z_at == "min") || (apply_to == "zmax" && z_at == "max")
						) {
							sphere(r = radius);
						} else {
							rotate = 
								(apply_to == "xmin" || apply_to == "xmax" || apply_to == "x") ? [0, 90, 0] : (
								(apply_to == "ymin" || apply_to == "ymax" || apply_to == "y") ? [90, 90, 0] :
								[0, 0, 0]
							);
							rotate(a = rotate)
							cylinder(h = diameter, r = radius, center = true);
						}
					}
				}
			}
		}
	}
}

module ring(
        h=1,
        od = 10,
        id = 5,
        de = 0.1
        ) 
{
    difference() {
        cylinder(h=h, r=od/2);
        translate([0, 0, -de])
            cylinder(h=h+2*de, r=id/2);
    }
}


// ring(2, 16, 12, 0.1 );

module torus2(r1, r2)
{
rotate_extrude() translate([r1,0,0]) circle(r2);
}

module oval_torus(inner_radius, thickness=[0, 0])
{
rotate_extrude() translate([inner_radius+thickness[0]/2,0,0]) ellipse(width=thickness[0], height=thickness[1]);
}


module fingerhole( rFinger = 8, rPipe = 50 ){
    difference(){
      union(){  
        torus2( rFinger, 1.1 );
        difference(){  
            color([0,1,0]) rotate([0,-90,0]) translate([ -(rPipe/2 + rFinger*1),0,0 ]) torus2(rPipe/2,  rFinger *1.3 );
             cylinder( rFinger*2.5, rFinger, rFinger, center=true);
        }
      }    
      union(){
        //translate ([0,0,0.5 ]) ring(1, (rFinger*2)-2, (rFinger*2)-4, 0.1 );
        translate ([0,0,0.5 ]) cylinder(1, 5, 5 );
        rotate([0,-90,0]) translate([ -rPipe,0,-0 ]) color([1,0,0]) cylinder( rFinger*4, rPipe, rPipe, center=true);
          
      }
    }
}


module fingerpinch( rFinger = 8, lPinch=16, wPinch=8, r1=30, r2=30, rPipe = 50 ){
    
    difference(){
      union(){  
        // translate([  0, lPinch/3/2+rFinger ,0 ]) roundedcube([ wPinch, lPinch /3 , 4 ], center=true, 0.5);  
          difference(){
              union(){
                          torus2( rFinger, 2 );
        rotate( [0,0,-r1] ) translate([  0, lPinch/2+rFinger ,0 ]) 
              roundedcube([ wPinch, lPinch, 4 ], center=true, 0.5);
              }      
        rotate( [0,0,-r1] ) translate([  0, lPinch/2+rFinger ,0 ]) 
              translate([ 0, 3 ,2 ]) cube([ 2, lPinch*1.5, 2 ], center=true);  
         }

        difference(){  
            color([0,1,0]) rotate([0,-90,0]) translate([ -(rPipe/2+ rFinger),2,-0 ]) torus2(rPipe/2,  rFinger *1.3 );
             cylinder( rFinger*2.5, rFinger, rFinger, center=true);
        }
      }    
      union(){
  
          
        translate ([0,0,0.5 ]) ring(1, (rFinger*2)-2, (rFinger*2)-4, 0.1 );
        rotate([0,-90,0]) translate([ -rPipe,0,-0 ]) color([1,0,0]) cylinder( rFinger*4, rPipe, rPipe, center=true);
          
      }
    }
}

module dualfingerhole( rFinger1 = 8, rFinger2=7, rPipe = 50 ){
    difference(){
      union(){  
        translate ([rFinger1+0.5,0,0 ])  torus2( rFinger1, 2 );
        translate ([-rFinger2-0.5,0,0 ])  torus2( rFinger2, 2 );
          
        difference(){  
           color([0,1,1]) rotate([0,-90,0]) translate([ -(rPipe/2+ rFinger1),2,-0 ]) torus2(rPipe/2,  rFinger1 *1.3 );
            union(){
             translate ([rFinger1+0.5,0,0 ]) cylinder( rFinger1*2.5, rFinger1, rFinger1, center=true);
             translate ([-rFinger2-0.5,0,0 ])cylinder( rFinger2*2.5, rFinger2, rFinger2, center=true);
        }
        }
      }    
      union(){
        translate ([rFinger1+0.5,0,0.5 ]) ring(1, (rFinger1*2)-2, (rFinger1*2)-4, 0.1 );
        translate ([-rFinger2-0.5,0,0.5 ]) ring(1, (rFinger2*2)-2, (rFinger2*2)-4, 0.1 );

        rotate([0,-90,0]) translate([ -rPipe,0,-0 ]) color([1,1,0]) cylinder( (rFinger1+rFinger2)*4, rPipe, rPipe, center=true);
          
      }
    }
}


module dualsidefingerhole( rFinger1 = 8, rFinger2=7, rotFinger1=1, rotFinger2=5, rPipe = 50 ){
    difference(){
      union(){  
        rotate([-rotFinger1,0,0]) translate ([0,rFinger1+1,0 ])  torus2( rFinger1, 2 );
        rotate([rotFinger2,0,0]) translate ([0,-rFinger2,0 ])  torus2( rFinger2, 2 );
          
        difference(){  
            color([0,1,1]) rotate([0,-90,0]) translate([ -(rPipe/2+ rFinger1),2,-0 ]) torus2(rPipe/2,  rFinger1 *1.3 );
            union(){
             rotate([- rotFinger1,0,0]) translate ([0, rFinger1+1,0 ]) cylinder( rFinger1*2.5, rFinger1, rFinger1, center=true);
             rotate([rotFinger2,0,0])  translate ([0,-rFinger2,0 ])cylinder( rFinger2*2.5, rFinger2, rFinger2, center=true);
        }
        }
      }    
      union(){
       rotate([- rotFinger1,0,0]) translate ( [0,rFinger1+1,0.5 ]) ring( 1, (rFinger1*2)-2, (rFinger1*2)-4, 0.1 );
       rotate([rotFinger2,0,0]) translate ([0,-rFinger2,0.5 ]) ring( 1, (rFinger2*2)-2, (rFinger2*2)-4, 0.1 );

        rotate([0,-90,0]) translate([ -rPipe,0,-0 ]) color([1,1,0]) cylinder( (rFinger1+rFinger2)*4, rPipe, rPipe, center=true);
          
      }
    }
}

module mouthpiece( w1=8, w2=10, h1=50 , h2=8, dr=3 ){
    // dr = durchmesser der Tube 
  difference(){
    hull(){
      translate([0,0,h1/10])cylinder( w1,w2,w2 ); 
      color([1,1,0]) translate([0,- w1/3,0])cylinder( 10,dr+1,dr+1 );   
      //translate([0,0,h1/2 + 2]) cube( [w2,w2,h1/2] , center=true); 
      translate([0,-w2/4,h1/2 + 2]) roundedcube( [w2,w2/2,h1] , center=true , 0.5); 
               
    }
    
    union(){
   rotate([0,-90,0]) translate([ h1,h1,-0 ]) color([1,1,0]) cylinder( (w1+w2), h1, h1, center=true );
        hull(){
       translate([0,-w2/4,h1])cube( [w2/1.5,w2/4,h1/2] , center=true );        
     // the hole for the pipe to the body
      color([1,1,0]) translate([0,- w1/3,0])cylinder( h1/2,dr,dr );   
        }
        
    }
  }
}

module mouthpieceWide( w1=8, w2=18, h1=50 , h2=8, dr=3 ){
    // dr = durchmesser der Tube 
  difference(){
    hull(){
      translate([0,0,h1/10])cylinder( w1,w2/1.8,w2/1.8 ); 
      color([1,1,0]) translate([0,- w1/3,0])cylinder( 10,dr+1,dr+1 );   
      //translate([0,0,h1/2 + 2]) cube( [w2,w2,h1/2] , center=true); 
      translate([0,-w2/4,h1/2 + 10]) roundedcube( [w2*1.1,w2/5,h1-5] , center=true , 0.5); 
               
    }
    
    union(){
        
        // Testschnitt: translate([0,-10,-5]) cube([100,100,100]);
        rotate([0,-90,0]) translate([ h1-4,h1+1.5,-0 ]) cylinder( (w1+w2), h1+1, h1+1, center=true );
        hull(){
            translate([0,-w2/4,h1])cube( [w2/1.5,w2/16,h1/2] , center=true );        
            // the hole for the pipe to the body
            color([1,1,0]) translate([0,- w1/3,0])cylinder( h1/2,dr,dr );   
        }
        
    }
  }
}


module pipeflansch( dr=3.55, rPipe=29.5 ){
    difference(){
      union(){  
        //#torus2( dr, 2 );

        difference(){  
            union(){
            translate([0,0,dr*4])   cylinder( dr*8, dr*1.8, dr*1.8, center=true);  
            hull(){
            color([0,1,0]) rotate([0,-90,0]) translate([ -(rPipe/2.4 +dr*0.5),0,0 ]) torus2(rPipe/2.4,  dr *1.8 );
               translate([-5,0,0])      cylinder( dr, rPipe/3, dr*1.5, center=true);  
               translate([5,0,0])      cylinder( dr, rPipe/3, dr*1.5, center=true);  
                cylinder( dr*4, rPipe/3, dr*1.5, center=true);    
            }    
        }
           cylinder( dr*2.5, dr, dr, center=true);
        }
      }   
      union(){
       // translate([0,0,dr*4])   cylinder( dr*8, dr*1.8, dr*1.8, center=true);  
        translate([0,0, dr*5.5 ]) cube([2,dr*6,dr*7.5], center=true );          
        translate ([0,0,0 ]) cylinder(100, dr, dr);
        rotate([0,-90,0]) translate([ -rPipe,0,-0 ]) color([1,0,0]) cylinder( dr*12, rPipe, rPipe, center=true);
          // 2 Löcher
          translate([ 0,-rPipe/2, 0 ])  rotate([35,0,0]) cylinder( 20, 1.2, 2.0 , center=true);
          translate([ 0, rPipe/2, 0 ])  rotate([-35,0,0]) cylinder( 20, 1.2, 2.0 , center=true);
          
      }
    }
}

module pipeflanschInner( dr=3, rPipe=28.5, drInner=2 ){
    difference(){
         translate([0,0, rPipe*2 ]) rotate([0,-90,0]) translate([ -rPipe,0,-0 ]) color([1,0,0]) cylinder( dr*6, rPipe, rPipe, center=true);  
        union(){
          translate([0,0, rPipe*1.2 ]) cube([ rPipe, rPipe*2, rPipe*2 ], center=true );
        }
    }
difference(){    
    translate ([0,0,-3]) cylinder( rPipe/1.4, drInner, drInner);
    translate ([0,0,0 ]) cylinder( rPipe, drInner-1, drInner-1);    
}
}


module pipeholder( dr=3.55, rPipe=50 ){
    difference(){
      union(){  
        //#torus2( dr, 2 );

        difference(){  
            union(){
            translate([0,0,dr ]) cube( [dr*2,dr*5,dr*2.5], center = true );      
            translate([0,0,dr*2.5 ])rotate([0,-90,0]) cylinder( dr*2, dr*2.5, dr*2.5, center=true );  
            translate([0,0,dr*2.5 ])rotate([0,-90,0]) cylinder( dr*3, dr*2, dr*2, center=true );  
            // translate([0,0,dr*4])   cylinder( dr*8, dr*1.8, dr*1.8, center=true);  
            hull(){
                color([0,1,0]) rotate([0,-90,0]) translate([ -(rPipe/2.4 +dr*0.5),0,0 ]) torus2(rPipe/2.4,  dr *1.8 );
               translate([0,0,0])      cylinder( dr, rPipe/3, dr*1.2, center=true);  
                cylinder( dr*2, rPipe/3, dr*1.2, center=true);    
            }    
        }
        
        
           // #cylinder( dr*2.5, dr, dr, center=true);
        }
      }   
      union(){
       // translate([0,0,dr*4])   cylinder( dr*8, dr*1.8, dr*1.8, center=true);  
        translate ([0,0,dr*2.5 ]) rotate([0,-90,0]) cylinder( 30, dr, dr, center=true);
        rotate([0,-90,0]) translate([ -rPipe,0,-0 ]) color([1,0,0]) cylinder( dr*12, rPipe, rPipe, center=true);
          
      }
    }
}

// Does not work because of the infill
module pipesplit( dr=2, dl=20 ){
    difference(){
        union(){
       rotate([0,0,0])        translate([0,0,dl/2])      cylinder( dl, dr, dr, center=true);  
       rotate([0,130,0])        translate([0,0,dl/2])      cylinder( dl, dr, dr, center=true);  
       rotate([0,230,0])        translate([0,0,dl/2 ])      cylinder( dl, dr, dr, center=true);  
        }

    union(){
       rotate([0,0,0])        translate([0,0,dl/2])      cylinder( dl+1, dr-1, dr-1, center=true);  
       rotate([0,130,0])        translate([0,0,dl/2])      cylinder( dl+1, dr-1, dr-1, center=true);  
       rotate([0,230,0])        translate([0,0,dl/2 ])      cylinder( dl+1, dr-1, dr-1, center=true);  
        }

        
    }
}

module pipesplitbox( dr=3.1, dl=20, rPipe=29.5 , secondhole=true ){
    difference(){
        union(){
          translate([0,0,dl/2])  cube ([dl,dl/2,dl], center=true );

        }

    union(){
          translate([dr+2,0, -dr/1.5])      cylinder( dl, dr, dr, center=true);  
          translate([dr+2,0, +dl +dr/1.5])      cylinder( dl, dr, dr, center=true);  
        
        
         if( secondhole ) {
          translate([-dr-2,0,dl])      cylinder( dl+1, dr, dr, center=true);  
        }else{
          translate([dr+2,0, 0])      cylinder( dl*2, dr, dr, center=true);  
          rotate([0,90,90]) translate([-dl/2,dl/4,0])      cylinder( dl+1, 1.5, 1.5, center=true);  
            
        }
// Bohrung
        rotate([0,90,90]) translate([-dl/2,0,0])      cylinder( dl+1, 1.5, 1.5, center=true);  

        translate([0,-2,dl/2])   cube ([dl-2,dl/2-2,dl/3], center=true);

     translate([ 0,rPipe + dl/5,10 ]) rotate([0,0,90])  color([1,0,0]) cylinder( dr*12, rPipe, rPipe, center=true);
        
        }
    }
}


module pipeholderbox( dr=3.1, dl=20, rPipe=29.5  ){
    difference(){
        union(){
          translate([0,0,dl/2])  cube ([dl,dl/2,dl/2], center=true );

        }

    union(){
          translate([dr+2,0, -dr/1.5])      cylinder( dl, dr, dr, center=true);  
          translate([dr+2,0, +dl +dr/1.5])      cylinder( dl, dr, dr, center=true);  
        
          translate([dr+2,0, 0])      cylinder( dl*2, dr, dr, center=true);  
          rotate([0,90,90]) translate([-dl/2,dl/4,0])      cylinder( dl+1, 1.5, 1.5, center=true);  
            
// Bohrung
        rotate([0,90,90]) translate([-dl/2,0,0])      cylinder( dl+1, 1.5, 1.5, center=true);  

        // translate([0,-2,dl/2])   cube ([dl-2,dl/2-2,dl/3], center=true);

     translate([ 0,rPipe + dl/5,10 ]) rotate([0,0,90])  color([1,0,0]) cylinder( dr*12, rPipe, rPipe, center=true);
        
        }

        
    }
}

// Boden aber mit ESP32 Quer eingebaut und USB-Port zur Seite
module pipeBottom2( rPipe=29.8, wPlatine=42 , rPipeInnter=24.5){
    
    // Platinen.Pfosten Dünn
    translate ([ 11.5, 25.5 ,5.5 ])  cylinder( 8, 0.8,0.8, center=true);
    translate ([ 11.5, -25.5 ,5.5 ]) cylinder( 8, 0.8,0.8, center=true);
    translate ([-11.5, -25.5 ,5.5 ])cylinder( 8, 0.8,0.8, center=true);
    translate ([-11.5, 25.5 ,5.5 ]) cylinder( 8, 0.8,0.8, center=true);
    // Platinenpfosten Dick
    translate ([ 11.5, 25.5 ,4.5 ])  cylinder( 4, 1.5,1.5, center=true);
    translate ([ 11.5, -25.5 ,4.5 ]) cylinder( 4, 1.5,1.5, center=true);
    translate ([-11.5, -25.5 ,4.5 ])cylinder( 4, 1.5,1.5, center=true);
    translate ([-11.5, 25.5 ,4.5 ]) cylinder( 4, 1.5,1.5, center=true);


    difference(){
        union(){

           union(){
                // translate([0,0,+7.5]) cube([27,55, 10], center=true );
                translate([0,27.5+5.2-2.5,+7.5])cube([27,5,13 ], center=true ); 
           }             
           // Runde Box für die ESP32
           difference(){
               translate([0,0,+6.5]) cylinder (12, rPipe+7, rPipe+4 , center = true);
               union(){
                    // Scheibe
                    translate([ 0, 0, +9]) cylinder( 12, rPipe+5, rPipe+2, center=true);

               }    
           }
           
           // Rand, ueberstand
           //translate([0,0,5]) cube([ rPipeInnter*2-1,12,10], center=true );
           // Platinenhalter Mittig
           translate([-rPipe+10,0,21]) cube([ 10,8,40], center=true );
           translate([+rPipe-10,0,21]) cube([ 10,8,40], center=true );
           // Torus Barrock
           difference(){
               union(){
                    translate ([0,0,14 ]) torus2( rPipe+2, 3.5 );
                    translate ([0,0,14 ]) cylinder( 3, rPipe+5,rPipe+5, center=true );
               }
               union(){
                    translate([ 0, 0, 12 ]) cylinder( 10, rPipe, rPipe, center=true);
               }
           }
           // Bohrungshuelsen
           rotate([0,0,45])  translate([  28,  0, 7.5 ]) cylinder( 14, 4,4, center=true);
           rotate([0,0,45])  translate([   0, 28, 7.5 ]) cylinder( 14, 4,4, center=true);
           rotate([0,0,45])  translate([ -28,  0, 7.5 ]) cylinder( 14, 4,4, center=true);
           rotate([0,0,45])  translate([   0,-28, 7.5 ]) cylinder( 14, 4,4, center=true);           
           rotate([0,0,0])   translate([  29,  0, 8 ]) cube([ 10, 3,13], center=true);
           rotate([0,0,180]) translate([  29,  0, 8 ]) cube([ 10, 3,13], center=true);

        }
        union(){
            // TESTCUT:
            rotate([ 0, 0, 45 ])cube([ 50, 50, 50 ]);
            
            difference(){
                translate([ 0, 0, 17 ]) cylinder( 14, rPipe+3, rPipe+3, center=true );
                translate([ 0, 0, 17 ]) cylinder( 14, rPipe  , rPipe  , center=true );
            }
            
            // Lüftungslöcher
            translate ([ 10, 10, 5.5 ]) cylinder( 20, 1.5,1.5, center=true );
            translate ([  0,  0, 5.5 ]) cylinder( 20, 1.5,1.5, center=true );
            translate ([-10, 10, 5.5 ]) cylinder( 20, 1.5,1.5, center=true );
            translate ([-10,-10, 5.5 ]) cylinder( 20, 1.5,1.5, center=true );
            translate ([ 10,-10, 5.5 ]) cylinder( 20, 1.5,1.5, center=true );
            translate ([  0,-20, 5.5 ]) cylinder( 20, 1.5,1.5, center=true );
            translate ([  0, 20, 5.5 ]) cylinder( 20, 1.5,1.5, center=true );
            
            // USB-Anschluss, 17mm über Mitte    
            translate([0,27.5+7.5,+7.5])cube([20,10,10 ], center=true ); 
            translate([0,27.5+7.5,+7.5])cube([15,40,8 ], center=true ); 
            
            // Zierring:
            difference(){ 
                cylinder(1, rPipe+2, rPipe+2 );
                cylinder(1, rPipe-5, rPipe-5 );
            }
            //translate([0,17,0]) cube([ 17.5,9,12], center=true );
            // Schalter    
            //translate([0,8,0]) cube([ 8,3,12], center=true );

            // ESP-32 Board / +8 !
            translate([0,0,+7.0]) cube([28,57, 11], center=true );
            
            // Platine 
            translate([0,-4,22]) cube([ wPlatine-4,10,40], center=true );
            translate([0,3,22]) cube([ wPlatine,8,40], center=true );
              
            // Bohrungen
            rotate([ 0,0,45]) translate([ 28,0,10 ])  cylinder( 20, 1.2,1.2, center=true);
            rotate([ 0,0,45]) translate([ 0,28,10 ])  cylinder( 20, 1.2,1.2, center=true);
            rotate([ 0,0,45]) translate([ -28,0,10 ]) cylinder( 20, 1.2,1.2, center=true);
            rotate([ 0,0,45]) translate([ 0,-28,10 ]) cylinder( 20, 1.2,1.2, center=true);      
            rotate([ 0,0,45]) translate([ 28,0, 5])   cylinder( 13, 2.5,2.5, center=true);
            rotate([ 0,0,45]) translate([ 0,28, 5 ])  cylinder( 13, 2.5,2.5, center=true);
            rotate([ 0,0,45]) translate([ -28,0, 5 ]) cylinder( 13, 2.5,2.5, center=true);
            rotate([ 0,0,45]) translate([ 0,-28, 5 ]) cylinder( 13, 2.5,2.5, center=true);
            // Pipe 
            // translate ([6.5,-34.1,0 ])cylinder( 10, 3.6, 3.6, center=true);
        }
    }
    
}

module pipeBottom2Ring(  rPipe=30.1 ){
    difference(){
        translate([ 0, 0, 13.7 ]) cylinder( 7.2, rPipe+2.5, rPipe+2.5, center=true );
        translate([ 0, 0, 13 ]) cylinder( 11, rPipe+1.5  , rPipe  , center=true );
    }
    
}


// Deckel um den Boden sauber zu verschließen
// Aktuell ist nur der USB-Anschluss sichtbar
module pipeBottom( rPipe=29.8, wPlatine=42 , rPipeInnter=24.5){
    difference(){
        union(){
            // Scheibe
            translate([0,0,0.9])cylinder( 1.8, rPipe, rPipe+4, center=true);
            // Rand, ueberstand
           translate([0,0,5]) cube([ rPipeInnter*2-1,12,10], center=true );
           // Platinenhalter Mittig
           translate([0,0,18]) cube([ rPipeInnter*2-1,8,34], center=true );
           // Torus Barrock
           difference(){
               union(){
                 translate ([0,0,3 ]) torus2( rPipe+2, 3.5 );
                 translate ([0,0,3 ]) cylinder( 3, rPipe+5,rPipe+5, center=true );
               }
               translate ([0,0,5 ])cylinder( 10, rPipe, rPipe, center=true);
           }
        }
        union(){
           // USB-Anschluss, 17mm über Mitte    
           translate([0,17,0]) cube([ 17.5,9,12], center=true );
           // Schalter    
           translate([0,8,0]) cube([ 8,3,12], center=true );

           // Platine 
           translate([0,-4,22]) cube([ wPlatine-2,10,40], center=true );
           translate([0,3,22]) cube([ wPlatine,8,40], center=true );
            
           // Bohrungen
           translate ([28,0,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([0,28,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([-28,0,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([0,-28,0 ])cylinder( 10, 1.2,1.2, center=true);      
      
           // Pipe 
           translate ([6.5,-34.1,0 ])cylinder( 10, 3.6, 3.6, center=true);
      

        }
    }
    
}

module pipeTop( rPipe=29.8, wPlatine=42 , rPipeInnter=24.5){
    difference(){
        union(){
            // Scheibe
            translate ([0,0,-2.8 ]) cylinder( 1.5, rPipe+2, rPipe+2, center=true);
            // Rand, ueberstand
            
            // Platinenhalter Mittig
           //translate([0,0,20]) cube([ rPipeInnter*2-1,8,40], center=true );
            // Torus Barrock
           translate([ 0,0,-3 ]) difference(){
               union(){
                 translate ([0,0,3 ]) torus2( rPipe+2, 3.5 );
                 translate ([0,0,3 ]) cylinder( 3, rPipe+5,rPipe+5, center=true );
               }
               translate ([0,0,5 ])cylinder( 10, rPipe, rPipe, center=true);
           }
           // OLED-Mount-Outer:                 
           // http://www.lcdwiki.com/0.96inch_OLED_Module_MC096VX
           // Offset 6.37 mm 
           // Offset wenn Display Mittig wäre:
           //  27.8-10.86 => 16,94 :2 => 8,47
           // Aktueller Offset 2.1+4.27 = 6,37 ==>> Korrektur um 2.1
           
            translate( [2.1,0,-2.5] ) union(){
               difference(){
                translate([0,0,1.5]) cube([ 32,30,4], center=true );
                translate([0,0,1.5]) cube([ 28,27.5,4], center=true );
                }
           
           // Bolzen für Display, je 2mm vom Rand entfernt, innen 2mm
             //   Aussen: H:27.8 B:27.3
                translate ([ 11.9, 11.65 ,2 ])cylinder( 5, 0.7, 0.7, center=true );       
                translate ([ -11.9, 11.65 ,2 ])cylinder( 5, 0.7, 0.7, center=true );       
                translate ([ 11.9, -11.65 ,2 ])cylinder( 5, 0.7, 0.7, center=true );       
                translate ([ -11.9, -11.65 ,2 ])cylinder( 5, 0.7, 0.7, center=true );       
            }

        }
        union(){
           // OLED   
           translate([0,0,0]) cube([ 12,23,10], center=true );
           // OLED-Mount Inner outcut 
            
           // LED    
           translate([0,18.0,0]) cylinder( 10, 1.6, 1.6, center=true ); 

           // Platine 
           //translate([0,-4,22]) cube([ wPlatine-2,10,40], center=true );
           //translate([0,4,22]) cube([ wPlatine,8,40], center=true );
            
           // Bohrungen
           translate ([28,0,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([0,28,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([-28,0,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([0,-28,0 ])cylinder( 10, 1.2,1.2, center=true);
            
           // Pipe 
           translate ([6.5,-34.1,0 ])cylinder( 10, 3.6, 3.6, center=true);
            
           //cube([20,120,20], center=true); 
        }
    }
    
}

module mpxholder(   ){
    
     difference(){
        union(){
            // MPX-Korpus Mittig
           translate([0,0,7.5]) cube([ 35,48,15], center=true ); // Höhe 15
            // Frontblende
           translate([17.5,0,7.5]) cube([ 3,70,16], center=true ); // Höhe 16
        }
        
        // Umriss des Drucksensors
        union(){
             difference(){   
               translate([-6,13,7.5]) cube([ 25,25,16], center=true );
               translate([ 10,-4,7.5]) cylinder( 16,29,29, center=true );   
             }
           
           // 2 Bohrungen 
             
             
            // Umriss des Drucksensors
           translate([ 0,-5,0  ]) rotate([0,0,30]) union(){ hull(){
               translate([0,12,7.5]) cylinder(24,3.6,3.6, center=true);
               translate([0,-12,7.5]) cylinder(24,3.6,3.6, center=true);
               translate([0,0,7.5]) cylinder(24,9.5,9.5, center=true);
               
           }
               translate([2,0,7.5]) cube([ 25,14,20], center=true );
               // Anschlüsse hinten
               translate([-10,0,4]) cube([ 30,12,10], center=true );
               // Anschlussschlauch front
               translate([10,0,9]) cube([ 30,20,10], center=true );
            }
        }
    }
}

// Modul, welches an die Röhre angeschraubt wird und dann nach oben strahlen soll
// Optisch erinnert es an die kleinen Roland AERO-Modelle
// You have to add Holes by Yourself !!
module loadspeakerCover( rPipe=29.8, wPlatine=42 , rPipeInner=24.5, pipelength=60, rot=35 ){
    difference(){
        union(){
            // Gehäuse
            
            rotate([ rot ,0,0])translate([0,0,+pipelength/2 ])cylinder( pipelength, rPipeInner, rPipeInner, center=true);
            // Rand, ueberstand
           translate([0,0,5]) cube([ rPipeInner*2-1,12,10], center=true );
           // Platinenhalter Mittig
           translate([0,0,18]) cube([ rPipeInner*2-1,8,34], center=true );
           // Torus Barrock
           rotate([rot ,0,0])translate([0,0,+pipelength ])  difference(){
               union(){
                 translate ([ 0,0,3 ]) torus2( rPipeInner, 3.5 );
                 translate ([ 0,0,3 ]) cylinder( 2, rPipeInner+3, rPipeInner+3, center=true );
               }
               translate ([0,0,5 ])cylinder( pipelength, rPipeInner-3, rPipeInner-3, center=true);
           }
        }
        union(){
            rotate([ rot ,0,0])translate([0,0,+pipelength/2 ])cylinder( pipelength-4, rPipeInner-2, rPipeInner-2, center=true);
           translate([ 0, 0, pipelength/2 ]) cylinder( pipelength*2, rPipe, rPipe , center=true);
            // USB-Anschluss, 17mm über Mitte    
           // #translate([0,17,0]) cube([ 17.5,9,12], center=true );
           // Schalter    
           // #translate([0,8,0]) cube([ 8,3,12], center=true );
            
           // Drill-holes
            /*
           translate ([28,0,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([0,28,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([-28,0,0 ])cylinder( 10, 1.2,1.2, center=true);
           translate ([0,-28,0 ])cylinder( 10, 1.2,1.2, center=true);      
      */
        }
    }
    
}

// ####### ############################################################################################
// #######
// ####### Activate the Models You need for Your Project and print them as often You need them
// #######
// ####### ############################################################################################

// ####### I used only pipeBottom2(), pipeTop() and mpxholder(), mouthpiece and no optional things... I hate waiting for the 3D-Prints...



// mpxholder();
// pipeBottom2();
// pipeBottom2Ring();
// pipeTop();

mouthpieceWide();
// mouthpiece();

// pipeholderbox();
// translate([ 0,20,0 ]) pipeholderbox();
// pipesplitbox( dr=3.1, dl=20, rPipe=29.5 , false );

// Optional to let it look a bit more like a SAX
// loadspeakerCover();



// ### Optional Gimmicks... perhaps the Fingerholes are interesting too provide a bit better orientation around the touchsensors ...

// translate([ 0,0,0 ]) pipeflansch( 3.1, 29.5 );
// fingerhole();
// GUT !! translate([ 0,0,0 ]) fingerhole(5.5,29.4);
// rotate([15,0,0])translate([ 30,0,0 ]) fingerpinch(8, 32, 9, 22, 30, 29.4);
// rotate([20,0,0])translate([ 0,0,0 ]) fingerhole(10,29.4);
// translate([ -15,0,0 ]) fingerhole(5,29.4);
// translate([ 15,0,0 ]) fingerhole(5,29.4);
// translate([ 60,0, 0 ]) fingerhole(6,50);
// translate([ -25,0, 0 ]) dualfingerhole(6,5,20);
// translate([ -20,0, 0 ]) dualfingerhole( 3.5, 4.5, 29.4 );

// pipeholder();
// DID NOT WORK !! translate([ 0,60,0 ]) pipeflanschInner();
// DID NOT WORK !! TOO THIN !!! translate([ 0,0,0 ]) pipesplit( 2.0, 15 );
