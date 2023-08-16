#uint32 magic_number                 #Unique 32-bit integer used to signal beginning of a VIO packet while parsing datastream 
int32 id                      #id number of tag
float32 size_m                      #size of tag in meterss
int64 timestamp_ns                  #timestamp at middle of the frame exposure in monotonic time

int32 loc_type                      #location type
float64[] T_tag_wrt_cam            #vector of length 3, representing tag position [x,y,z]
float64[] R_tag_to_cam             #vector of length 9, representing flattened 3x3 rotation matrix,  
float64[] T_tag_wrt_fixed          #vector of length 3, representing tag position [x,y,z]
float64[] R_tag_to_fixed           #vector of length 9, representing flattened 3x3 rotation matrix

int32 reserved                      #reserved
