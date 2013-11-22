-- Wireshark_JAUS_dissector.lua , Wireshark Lua dissector script for IGVC JAUS
-- dissects the JAUS Protocol V1.000
--
-- 2013 Nov 13, University of Detroit Mercy
--              Advanced Mobile Robotics Lab , Cheng-Lung Lee, Hong-Yi Zhang

-- Notes : for some reason  jaus_proto.fields don't work (on Mac, no idea for windows and linux)...,
--         1 byte difference UDP data and as5669a, ie UDP is 18 byte as5669a package size is 17
--         currently I just ignore the HC flag & number & length ...seems they are all zero ...
-- Output Formate
--
-- JAUS Data (xx bytes)
--	 Data:12345678901234567890
--
--  Message ID:  0x2B00 (QueryIdentification)
--  Message   :
--
--
-- Data size is (16 Bytes + Payload)
--JAUS SDP (as5669a):
--  Message type  :
--  HC flags:
--  Data Size:
--  HC Number:
--  HC Length:
--  Priority:
--  B'cast:
--  Ack/Nak:
--  Data Flags:
--  Destination ID:
--  Source      ID:
--  Payload (xx Bytes):
--  Sequence Number:


do

packetnum = -1
-- Create a new dissector
jaus_proto = Proto("JAUS","Jaus Protocol")

MessageIDs = { [0x2B00] = "QueryIdentification", [0x2B01] = "QueryConfiguration", [0x2B02] = "QuerySubsystemList"}
-- Create the protocol fields
local
f = jaus_proto.fields
f.alldata       = ProtoField.bytes("jaus_proto.alldata", "Data")
f.messagetype   = ProtoField.bytes  ("jaus_proto.mtype"     , "Message Type")
-- HC flag
f.datasize      = ProtoField.uint16 ("jaus_proto.datasize"  , "Data Size")
-- HC Number
-- HC Length
f.pbad          = ProtoField.bytes  ("jaus_proto.pbad"      , "Priority, B'cast, Ack/Nak,Data Flags")
f.destinationid = ProtoField.bytes  ("jaus_proto.d_id"      , "Destination ID")
f.sourceid      = ProtoField.bytes  ("jaus_proto.s_id"      , "Source ID")
f.payload       = ProtoField.bytes  ("jaus_proto.payload"   , "Payload")
f.messageid     = ProtoField.uint16("jaus_proto.messageid", "MessageID", base.HEX, MessageIDs)
f.seq_number    = ProtoField.uint16 ("jaus_proto.seq_number", "Sequence Number")
-- A initialization routine
local packet_counter
function jaus_proto.init ()
packet_counter = 0
end

local const PI=3.14159265358979323846;
local const ByteRange       = 255.0;
local const UInt64HalfRange = 1.8446744073709552E+19;
local const Int64HalfRange  = 9.2233720368547758E+18;
local const UInt32Range     = 4294967295.0;
local const Int32Range      = 4294967294.0;
local const UInt16Range     = 65535.0;
local const Int16Range      = 65534.0;
local const Epsilon = .00000000000000000000001;
local int_l = 4
local short_l = 2
local char_l  = 1
local byte_l  = 1
local lower_sl = 0
local upper_sl = 327.67
local pv_bytes = 0


-- create a function to dissect it
function jaus_proto.dissector(buffer,pinfo,tree)
   pinfo.cols.protocol = "Jaus"
--   pinfo.cols.info:set("Jaus test info");

local as5669a_length=buffer:len()

-- Adding fields to the tree
local subtree = tree:add (jaus_proto, buffer(), "JAUS Traffic("..buffer:len().." bytes)")
subtree:add_le(buffer(), "JAUS as5669a traffic")
subtree:add_le(buffer(1,1),"Message Type  :" .. buffer(1,1):le_uint())
local payloadsize=buffer(2,2):le_uint()
subtree:add_le(buffer(2,2),"Data Size     :" .. buffer(2,2):le_uint())
subtree:add_le(buffer(4,1),"PBAD Flags    :" .. buffer(4,1):le_uint())
subtree:add_le(buffer(5,4),"Destination ID:" .. buffer(5,4):le_uint() .. "(" .. buffer(7,1):uint().. "-" .. buffer(6,1):uint().. "-" .. buffer(5,1):uint().. ")")
subtree:add_le(buffer(9,4),"Source ID     :" .. buffer(9,4):le_uint() .. "(" .. buffer(11,1):uint().. "-" .. buffer(10,1):uint().. "-" .. buffer(9,1):uint().. ")")
--subtree:add_le(buffer(13,3),"Payload: " .. buffer(13,3):bytes())
--subtree:add_le(buffer(13,2),"Message ID: " .. buffer(13,2):le_uint())
--local total_data_size=buffer(2,2):le_uint()
if as5669a_length-1 >=16 then
	local data=buffer(13,2):le_uint()
	local messageid=data
	local id_str = "unkown"
	--PV table


	if messageid==0x0001 then id_str="SetAuthority"                   ; pv_bytes= 0
		elseif messageid==0x0002 then id_str="Shutdown"                   ; pv_bytes= 0
		elseif messageid==0x0003 then id_str="Standby"                    ; pv_bytes= 0
		elseif messageid==0x0004 then id_str="Resume"                     ; pv_bytes= 0
		elseif messageid==0x0005 then id_str="Reset"                      ; pv_bytes= 0
		elseif messageid==0x0006 then id_str="SetEmergency"               ; pv_bytes= 0
		elseif messageid==0x0007 then id_str="ClearEmergency"             ; pv_bytes= 0
		elseif messageid==0x000D then id_str="RequestControl"             ; pv_bytes= 0
		elseif messageid==0x000E then id_str="ReleaseControl"             ; pv_bytes= 0
		elseif messageid==0x000F then id_str="ConfirmControl"             ; pv_bytes= 0
		elseif messageid==0x0010 then id_str="RejectControl"              ; pv_bytes= 0
		elseif messageid==0x0011 then id_str="SetTime"                    ; pv_bytes= 1
		elseif messageid==0x01F0 then id_str="CreateEvent"                ; pv_bytes= 0
		elseif messageid==0x01F1 then id_str="UpdateEvent"                ; pv_bytes= 0
		elseif messageid==0x01F2 then id_str="CancelEvent"                ; pv_bytes= 0
		elseif messageid==0x01F3 then id_str="ConfirmEventRequest"        ; pv_bytes= 0
		elseif messageid==0x01F4 then id_str="RejectEventRequest"         ; pv_bytes= 1
		elseif messageid==0x0B00 then id_str="RegisterServices"           ; pv_bytes= 0
		elseif messageid==0x2001 then id_str="QueryAuthority"             ; pv_bytes= 0
		elseif messageid==0x2002 then id_str="QueryStatus"                ; pv_bytes= 0
		elseif messageid==0x2003 then id_str="QueryTimeout"               ; pv_bytes= 0
		elseif messageid==0x2011 then id_str="QueryTime"                  ; pv_bytes= 1
		elseif messageid==0x200D then id_str="QueryControl"               ; pv_bytes= 0
		elseif messageid==0x21F0 then id_str="QueryEvents"                ; pv_bytes= 0
		elseif messageid==0x2202 then id_str="QueryHeartbeatPulse"        ; pv_bytes= 0
		elseif messageid==0x2B00 then id_str="QueryIdentification"        ; pv_bytes= 0
		elseif messageid==0x2B01 then id_str="QueryConfiguration"         ; pv_bytes= 0
		elseif messageid==0x2B02 then id_str="QuerySubsystemList"         ; pv_bytes= 0
		elseif messageid==0x2B03 then id_str="QueryServices"              ; pv_bytes= 0
		elseif messageid==0x4001 then id_str="ReportAuthority"            ; pv_bytes= 0
		elseif messageid==0x4002 then id_str="ReportStatus"               ; pv_bytes= 0
		elseif messageid==0x4003 then id_str="ReportTimeout"              ; pv_bytes= 0
		elseif messageid==0x4011 then id_str="ReportTime"                 ; pv_bytes= 1
		elseif messageid==0x400D then id_str="ReportControl"              ; pv_bytes= 0
		elseif messageid==0x41F0 then id_str="ReportEvents"               ; pv_bytes= 0
		elseif messageid==0x41F1 then id_str="Event"                      ; pv_bytes= 0
		elseif messageid==0x4202 then id_str="ReportHeartbeatPulse"       ; pv_bytes= 0
		elseif messageid==0x4B00 then id_str="ReportIdentification"       ; pv_bytes= 0
		elseif messageid==0x4B01 then id_str="ReportConfiguration"        ; pv_bytes= 0
		elseif messageid==0x4B02 then id_str="ReportSubsystemList"        ; pv_bytes= 0
		elseif messageid==0x4B03 then id_str="ReportServices"             ; pv_bytes= 0
		elseif messageid==0x0402 then id_str="SetGlobalPose"              ; pv_bytes= 2
		elseif messageid==0x0403 then id_str="SetLocalPose"               ; pv_bytes= 2
		elseif messageid==0x0405 then id_str="SetWrenchEffort"            ; pv_bytes= 2
		elseif messageid==0x0407 then id_str="SetGlobalVector"            ; pv_bytes= 1
		elseif messageid==0x0408 then id_str="SetLocalVector"             ; pv_bytes= 1
		elseif messageid==0x040A then id_str="SetTravelSpeed"             ; pv_bytes= 0
		elseif messageid==0x040C then id_str="SetGlobalWaypoint"          ; pv_bytes= 1
		elseif messageid==0x040D then id_str="SetLocalWaypoint"           ; pv_bytes= 1
		elseif messageid==0x040F then id_str="SetGlobalPathSegment"       ; pv_bytes= 1
		elseif messageid==0x0410 then id_str="SetLocalPathSegment"        ; pv_bytes= 1
		elseif messageid==0x0412 then id_str="SetGeomagneticProperty"     ; pv_bytes= 0
		elseif messageid==0x0415 then id_str="SetVelocityCommand"         ; pv_bytes= 1
		elseif messageid==0x0416 then id_str="SetAccelerationLimit"       ; pv_bytes= 1
		elseif messageid==0x041A then id_str="SetElement"                 ; pv_bytes= 0
		elseif messageid==0x041B then id_str="DeleteElement"              ; pv_bytes= 0
		elseif messageid==0x041C then id_str="ConfirmElementRequest"      ; pv_bytes= 0
		elseif messageid==0x041D then id_str="RejectElementRequest"       ; pv_bytes= 0
		elseif messageid==0x041E then id_str="ExecuteList"                ; pv_bytes= 1
		elseif messageid==0x2402 then id_str="QueryGlobalPose"            ; pv_bytes= 2
		elseif messageid==0x2403 then id_str="QueryLocalPose"             ; pv_bytes= 2
		elseif messageid==0x2404 then id_str="QueryVelocityState"         ; pv_bytes= 2
		elseif messageid==0x2405 then id_str="QueryWrenchEffort"          ; pv_bytes= 2
		elseif messageid==0x2407 then id_str="QueryGlobalVector"          ; pv_bytes= 1
		elseif messageid==0x2408 then id_str="QueryLocalVector"           ; pv_bytes= 1
		elseif messageid==0x240A then id_str="QueryTravelSpeed"           ; pv_bytes= 0
		elseif messageid==0x240C then id_str="QueryGlobalWaypoint"        ; pv_bytes= 1
		elseif messageid==0x240D then id_str="QueryLocalWaypoint"         ; pv_bytes= 1
		elseif messageid==0x240F then id_str="QueryGlobalPathSegment"     ; pv_bytes= 1
		elseif messageid==0x2410 then id_str="QueryLocalPathSegment"      ; pv_bytes= 1
		elseif messageid==0x2412 then id_str="QueryGeomagneticProperty"   ; pv_bytes= 0
		elseif messageid==0x2415 then id_str="QueryVelocityCommand"       ; pv_bytes= 1
		elseif messageid==0x2416 then id_str="QueryAccelerationLimit"     ; pv_bytes= 1
		elseif messageid==0x2417 then id_str="QueryAccelerationState"     ; pv_bytes= 1
		elseif messageid==0x241A then id_str="QueryElement"               ; pv_bytes= 0
		elseif messageid==0x241B then id_str="QueryElementList"           ; pv_bytes= 0
		elseif messageid==0x241C then id_str="QueryElementCount"          ; pv_bytes= 0
		elseif messageid==0x241E then id_str="QueryActiveElement"         ; pv_bytes= 0
		elseif messageid==0x4402 then id_str="ReportGlobalPose"           ; pv_bytes= 0
		elseif messageid==0x4403 then id_str="ReportLocalPose"            ; pv_bytes= 0
		elseif messageid==0x4404 then id_str="ReportVelocityState"        ; pv_bytes= 2
		elseif messageid==0x4405 then id_str="ReportWrenchEffort"         ; pv_bytes= 0
		elseif messageid==0x4407 then id_str="ReportGlobalVector"         ; pv_bytes= 0
		elseif messageid==0x4408 then id_str="ReportLocalVector"          ; pv_bytes= 0
		elseif messageid==0x440A then id_str="ReportTravelSpeed"          ; pv_bytes= 0
		elseif messageid==0x440C then id_str="ReportGlobalWaypoint"       ; pv_bytes= 0
		elseif messageid==0x440D then id_str="ReportLocalWaypoint"        ; pv_bytes= 0
		elseif messageid==0x440F then id_str="ReportGlobalPathSegment"    ; pv_bytes= 0
		elseif messageid==0x4410 then id_str="ReportLocalPathSegment"     ; pv_bytes= 0
		elseif messageid==0x4412 then id_str="ReportGeomagneticProperty"  ; pv_bytes= 0
		elseif messageid==0x4415 then id_str="ReportVelocityCommand"      ; pv_bytes= 0
		elseif messageid==0x4416 then id_str="ReportAccelerationLimit"    ; pv_bytes= 0
		elseif messageid==0x4417 then id_str="ReportAccelerationState"    ; pv_bytes= 2
		elseif messageid==0x441A then id_str="ReportElement"              ; pv_bytes= 0
		elseif messageid==0x441B then id_str="ReportElementList"          ; pv_bytes= 0
		elseif messageid==0x441C then id_str="ReportElementCount"         ; pv_bytes= 0
		elseif messageid==0x441E then id_str="ReportActiveElement"        ; pv_bytes= 0
	end

	--info(id_str)
	subtree:add(buffer(13,2),"Message ID: " .. string.format('%04X',data) .. " [" .. id_str .. "]")
	pinfo.cols.info:set("Jaus Message ID:" .. string.format('%04X',data)  .. " [" .. id_str .. "]");

	if as5669a_length-1 >=17 then
		if pv_bytes > 0 then
			local data=buffer(15,pv_bytes):le_uint()
			subtree:add(buffer(15,pv_bytes),"Message PV: " .. string.format('%X',data))
		end
		if (as5669a_length-1-pv_bytes) >= 17 then
			subtree:add(buffer(15+pv_bytes,as5669a_length-(15+pv_bytes)-2),"Message Data: " )
		end

	end
	subtree:add_le(buffer(as5669a_length-2,2),"Sequence Number: " .. buffer(as5669a_length-2,2):le_uint())

--Message decoding
	if messageid==0x0001 then id_str="SetAuthority"
		elseif messageid==0x0002 then id_str="Shutdown"
		elseif messageid==0x0003 then id_str="Standby"
		elseif messageid==0x0004 then id_str="Resume"
		elseif messageid==0x0005 then id_str="Reset"
		elseif messageid==0x0006 then id_str="SetEmergency"
		elseif messageid==0x0007 then id_str="ClearEmergency"
		elseif messageid==0x000D then id_str="RequestControl"
		local AuthorityCode = buffer(15,byte_l):le_uint()
		subtree:add(buffer(15,byte_l),"AuthorityCode: " .. string.format('%x',AuthorityCode))

		elseif messageid==0x000E then id_str="ReleaseControl"
		elseif messageid==0x000F then id_str="ConfirmControl"
		local ResponseCode = buffer(15,byte_l):le_uint()
		--local ResponseCode_mask = 0xFF
		interpretation = {"value_enum 0 CONTROL ACCEPTED","value_enum 1 NOT AVAILABLE","value_enum 2 INSUFFICIENT AUTHORITY"}
		--interpretation_i = bit.band(ResponseCode,ResponseCode_mask)
		subtree:add(buffer(15,byte_l),"ResponseCode: " .. string.format('%s',interpretation[ResponseCode+1]))




		elseif messageid==0x0010 then id_str="RejectControl"
		elseif messageid==0x0011 then id_str="SetTime"
		elseif messageid==0x01F0 then id_str="CreateEvent"
		elseif messageid==0x01F1 then id_str="UpdateEvent"
		elseif messageid==0x01F2 then id_str="CancelEvent"
		elseif messageid==0x01F3 then id_str="ConfirmEventRequest"
		elseif messageid==0x01F4 then id_str="RejectEventRequest"
		elseif messageid==0x0B00 then id_str="RegisterServices"
		elseif messageid==0x2001 then id_str="QueryAuthority"
		elseif messageid==0x2002 then id_str="QueryStatus"
		elseif messageid==0x2003 then id_str="QueryTimeout"
		elseif messageid==0x2011 then id_str="QueryTime"
		elseif messageid==0x200D then id_str="QueryControl"
		elseif messageid==0x21F0 then id_str="QueryEvents"
		elseif messageid==0x2202 then id_str="QueryHeartbeatPulse"
		elseif messageid==0x2B00 then id_str="QueryIdentification"
		local Query_No   = buffer(15,byte_l):le_uint()
		local Query_Mask = 0xFF
		local Query_i = bit.band(Query_No,Query_Mask)
		local Query_type= {"0:Reserved", "System Identification", "2:Subsystem Configuration", "3:Node Configuration", "4-255:Reserved" }
		subtree:add(buffer(15,byte_l),"Query_type: " .. string.format('%s',Query_type[Query_i+1]))

		elseif messageid==0x2B01 then id_str="QueryConfiguration"
		elseif messageid==0x2B02 then id_str="QuerySubsystemList"
		elseif messageid==0x2B03 then id_str="QueryServices"
		local PV1 = buffer(15,1):le_uint()
		local PV2 = buffer(16,1):le_uint()
		if (PV1 == 1) then
		subtree:add(buffer(15,1),"Node_ID: " .. string.format('%x',PV1))
		end

		if (PV2 == 0) then
		subtree:add(buffer(16,1),"Component_ID: " .. string.format('%x',PV2))
		end




		elseif messageid==0x4001 then id_str="ReportAuthority"
		elseif messageid==0x4002 then id_str="ReportStatus"

		local status_encode={"0 INIT","1 READY","2 STANDBY","3 SHUTDOWN","4 FAILURE","5 EMERGENCY"}
		local status_case=buffer(15,1):le_uint()
		subtree:add_le(buffer(15,1),"Vehicle_status: " .. string.format('%s',status_encode[status_case+1]))




		elseif messageid==0x4003 then id_str="ReportTimeout"
		elseif messageid==0x4011 then id_str="ReportTime"
		elseif messageid==0x400D then id_str="ReportControl"

		local SubsystemID = buffer(15,2):le_uint()
		subtree:add_le(buffer(15,2),"SubsystemID: " .. string.format('%s',SubsystemID))
		local NodeID = buffer(17,1):le_uint()
		subtree:add_le(buffer(17,1),"NodeID: " .. string.format('%s',NodeID))
		local ComponentID = buffer(18,1):le_uint()
		subtree:add_le(buffer(18,1),"ComponentID: " .. string.format('%s',ComponentID))
		local AuthorityCode = buffer(19,1):le_uint()
		subtree:add_le(buffer(19,1),"AuthorityCode: " .. string.format('%s',AuthorityCode))

		elseif messageid==0x41F0 then id_str="ReportEvents"
		elseif messageid==0x41F1 then id_str="Event"
		elseif messageid==0x4202 then id_str="ReportHeartbeatPulse"
		elseif messageid==0x4B00 then id_str="ReportIdentification"

		local data_l=buffer(18,1):le_uint()
		local data_s={}
		for i=0,data_l-1 do data_s[i+1]=buffer(19+i,1) end
		a = bytestostring2(data_s,data_l)
		subtree:add_le(buffer(19,data_l),"Identification: " .. string.format('%s',a))
		elseif messageid==0x4B01 then id_str="ReportConfiguration"
		elseif messageid==0x4B02 then id_str="ReportSubsystemList"
		elseif messageid==0x4B03 then id_str="ReportServices"

		local service_No_pose = 20
		local service_No = buffer(service_No_pose,1):le_uint()
		local service_len_pose={}
		local service_len = {}

		for i=1, service_No do if i == 1 then
				service_len_pose[i] = service_No_pose + 1
				service_len[i] = buffer(service_len_pose[i],1):le_uint()
			else
				service_len_pose[i] = service_len_pose[i-1] + service_len[i-1] + 3
				service_len[i] = buffer(service_len_pose[i],1):le_uint()
			end

			local data_s = {}
			for s=0,service_len[i] do data_s[s+1]=buffer(service_len_pose[i]+s+1,1) end
			data_s = bytestostring2(data_s,service_len[i])
			subtree:add_le(buffer(service_len_pose[i]+1,service_len[i]),"Servers [" .. string.format('%i',i).."] information:" .. string.format('%s',data_s))
		end

		elseif messageid==0x0402 then id_str="SetGlobalPose"
		elseif messageid==0x0403 then id_str="SetLocalPose"
		local x_pose  = 17
		local scale_x = buffer(x_pose,int_l):le_uint()
		x  = UInt32Toscale(scale_x,low_x_y_z,high_x_y_z)

		local scale_y = buffer(x_pose+int_l,int_l):le_uint()
		y  = UInt32Toscale(scale_y,low_x_y_z,high_x_y_z)

		local scale_z = buffer(x_pose+int_l+int_l,int_l):le_uint()
		z  = UInt32Toscale(scale_z,low_x_y_z,high_x_y_z)

		local scale_yaw = buffer(x_pose+int_l+int_l+int_l,short_l):le_uint()
		yaw  = UInt16Toscale(scale_yaw,low_yaw,high_yaw)

		subtree:add_le(buffer(x_pose,int_l),"x:   " .. string.format('%8.2f',x))

		subtree:add_le(buffer(x_pose+int_l,int_l),"y:   " .. string.format('%8.2f',y))

		subtree:add_le(buffer(x_pose+int_l+int_l,int_l),"z:   " .. string.format('%8.2f',z))

		subtree:add_le(buffer(x_pose+int_l+int_l+int_l,short_l),"yaw: " .. string.format('%8.2f',yaw))

		elseif messageid==0x0405 then id_str="SetWrenchEffort"
		elseif messageid==0x0407 then id_str="SetGlobalVector"
		elseif messageid==0x0408 then id_str="SetLocalVector"
		elseif messageid==0x040A then id_str="SetTravelSpeed"
		elseif messageid==0x040C then id_str="SetGlobalWaypoint"
		elseif messageid==0x040D then id_str="SetLocalWaypoint"
		elseif messageid==0x040F then id_str="SetGlobalPathSegment"
		elseif messageid==0x0410 then id_str="SetLocalPathSegment"
		elseif messageid==0x0412 then id_str="SetGeomagneticProperty"
		elseif messageid==0x0415 then id_str="SetVelocityCommand"
		elseif messageid==0x0416 then id_str="SetAccelerationLimit"
		elseif messageid==0x041A then id_str="SetElement"

		local request_id  = buffer(15,1):le_uint()
		local counter     = buffer(16,1):le_uint()
		local packet_pose = 17
		local data_l      = 22
		local const low_x_y_z  = -100000
		local const high_x_y_z =  100000
		local const low_t        = 0
		local const high_t       = 100
		for i=1,counter do

		element_id  = buffer(packet_pose+(i-1)*data_l,short_l):le_uint()
		previous_id = buffer(packet_pose+short_l+(i-1)*data_l,short_l):le_uint()
		next_id     = buffer(packet_pose+short_l+short_l+(i-1)*data_l,short_l):le_uint()
		enum_bit    = buffer(packet_pose+short_l+short_l+short_l+(i-1)*data_l,char_l):le_uint()
		count_field = buffer(packet_pose+short_l+short_l+short_l+char_l+(i-1)*data_l,short_l):le_uint()
		msg_id      = buffer(packet_pose+short_l+short_l+short_l+short_l+char_l+(i-1)*data_l,short_l):le_uint()
		pv             = buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+(i-1)*data_l,char_l):le_uint()
		scale_x        = buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+char_l+(i-1)*data_l,int_l):le_uint()
		x              = UInt32Toscale(scale_x,low_x_y_z,high_x_y_z)
		scale_y        = buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+char_l+int_l+(i-1)*data_l,int_l):le_uint()
		y              = UInt32Toscale(scale_y,low_x_y_z,high_x_y_z)
		scale_tolerance= buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+char_l+int_l+int_l+(i-1)*data_l,short_l):le_uint()
		tolerance      = UInt16Toscale(scale_tolerance,low_t,high_t)

		subtree:add_le(buffer(packet_pose+(i-1)*data_l,short_l)                                                                  ,"element_id :   " ..string.format('%i',i).. "   " .. string.format('%i',element_id))
		subtree:add_le(buffer(packet_pose+short_l+(i-1)*data_l,short_l)                                                          ,"previous_id:   " .."    " .. string.format('%i',previous_id))
		subtree:add_le(buffer(packet_pose+short_l+short_l+(i-1)*data_l,short_l)                                                  ,"next_id    :   " .."    " .. string.format('%i',next_id))
		subtree:add_le(buffer(packet_pose+short_l+short_l+short_l+(i-1)*data_l,short_l)                                          ,"enum_bit   :   " .."    " .. string.format('%i',enum_bit))
		subtree:add_le(buffer(packet_pose+short_l+short_l+short_l+char_l+(i-1)*data_l,short_l)                                   ,"count_field:   " .."    " .. string.format('%i',count_field))
		subtree:add_le(buffer(packet_pose+short_l+short_l+short_l+short_l+char_l+(i-1)*data_l,short_l)                           ,"msg_id     :   " .."    " .. string.format('%i',msg_id))
		subtree:add_le(buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+(i-1)*data_l,short_l)                   ,"pv         :   " .."    " .. string.format('%i',pv))
		subtree:add_le(buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+char_l+(i-1)*data_l,short_l)            ,"x          :   " .. string.format('%8.2f',x))
		subtree:add_le(buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+char_l+int_l+(i-1)*data_l,short_l)      ,"y          :   " .. string.format('%8.2f',y))
		subtree:add_le(buffer(packet_pose+short_l+short_l+short_l+short_l+short_l+char_l+char_l+int_l+int_l+(i-1)*data_l,short_l),"tolerance  :   " .. string.format('%8.2f',tolerance))

		end

		elseif messageid==0x041B then id_str="DeleteElement"
		elseif messageid==0x041C then id_str="ConfirmElementRequest"

		local 	req_id = buffer(15,1):le_uint()
				subtree:add_le(buffer(15,1),"req_id:   " .. string.format('%i',req_id))

		elseif messageid==0x041D then id_str="RejectElementRequest"
		elseif messageid==0x041E then id_str="ExecuteList"

		local scale_speed = buffer(16,short_l):le_uint()
		local UID_start = buffer(16+short_l,short_l):le_uint()
		speed = UInt16Toscale(scale_speed,lower_sl,upper_sl)

		subtree:add_le(buffer(16,short_l),"speed: " .. string.format('%s',speed))
		subtree:add_le(buffer(16+short_l,short_l),"UID_start: " .. string.format('%i',UID_start))

		elseif messageid==0x2402 then id_str="QueryGlobalPose"
		elseif messageid==0x2403 then id_str="QueryLocalPose"

		local PV = buffer(15,2):le_uint()

		local table_local_pose = {"X","Y","Z","PositionRms","Roll","Pitch","Yaw","AttitudeRms","TimeStamp"}
		for i=1,9 do
		    if (bit.band(2^(i-1),PV)>0) then
				subtree:add(buffer(15,2),"Message PV element: " .. string.format('%s',table_local_pose[i]))
			end

		end


		elseif messageid==0x2404 then id_str="QueryVelocityState"
		local PV = buffer(15,2):le_uint()
		local table_velocity = {"vx","vy","vz","velocity_RMS","roll_rate","pitch_rate","yaw_rate","rate_RMS","TimeStamp"}
		for i=1,3 do
		if (bit.band(2^(i-1),PV)>0) then
				subtree:add(buffer(15,2),"Message PV element: " .. string.format('%s',table_velocity[i]))
			end

		end

		elseif messageid==0x2405 then id_str="QueryWrenchEffort"
		elseif messageid==0x2407 then id_str="QueryGlobalVector"
		elseif messageid==0x2408 then id_str="QueryLocalVector"
		elseif messageid==0x240A then id_str="QueryTravelSpeed"
		elseif messageid==0x240C then id_str="QueryGlobalWaypoint"
		elseif messageid==0x240D then id_str="QueryLocalWaypoint"
		elseif messageid==0x240F then id_str="QueryGlobalPathSegment"
		elseif messageid==0x2410 then id_str="QueryLocalPathSegment"
		elseif messageid==0x2412 then id_str="QueryGeomagneticProperty"
		elseif messageid==0x2415 then id_str="QueryVelocityCommand"
		elseif messageid==0x2416 then id_str="QueryAccelerationLimit"
		elseif messageid==0x2417 then id_str="QueryAccelerationState"
		elseif messageid==0x241A then id_str="QueryElement"
		elseif messageid==0x241B then id_str="QueryElementList"
		elseif messageid==0x241C then id_str="QueryElementCount"
		elseif messageid==0x241E then id_str="QueryActiveElement"
		elseif messageid==0x4402 then id_str="ReportGlobalPose"
		elseif messageid==0x4403 then id_str="ReportLocalPose"

		local PV = buffer(15,2):le_uint()
		if PV == 0x147 then
		local x_pose = 17
		local const low_x_y_z  = -100000
		local const high_x_y_z =  100000
		local const low_yaw  =  -PI
		local const high_yaw =   PI



		local scale_x = buffer(x_pose,int_l):le_uint()
		x  = UInt32Toscale(scale_x,low_x_y_z,high_x_y_z)

		local scale_y = buffer(x_pose+int_l,int_l):le_uint()
		y  = UInt32Toscale(scale_y,low_x_y_z,high_x_y_z)

		local scale_z = buffer(x_pose+int_l+int_l,int_l):le_uint()
		z  = UInt32Toscale(scale_z,low_x_y_z,high_x_y_z)

		local scale_yaw = buffer(x_pose+int_l+int_l+int_l,short_l):le_uint()
		yaw  = UInt16Toscale(scale_yaw,low_yaw,high_yaw)

		local Time_stamp = buffer(x_pose+int_l+int_l+int_l+short_l,int_l):le_uint()
		Time = Timestamp2time(Time_stamp)

		subtree:add_le(buffer(x_pose,int_l),"x:   " .. string.format('%8.2f',x))

		subtree:add_le(buffer(x_pose+int_l,int_l),"y:   " .. string.format('%8.2f',y))

		subtree:add_le(buffer(x_pose+int_l+int_l,int_l),"z:   " .. string.format('%8.2f',z))

		subtree:add_le(buffer(x_pose+int_l+int_l+int_l,short_l),"yaw: " .. string.format('%8.2f',yaw))

		subtree:add_le(buffer(x_pose+int_l+int_l+int_l+short_l,int_l),"Time Stamp = Day: " .. string.format('%i',Time[1]).." Time: " .. string.format('%02i',Time[2]) ..":" .. string.format('%02i',Time[3])..":" .. string.format('%02i',Time[4]).."." .. string.format('%03i',Time[5]))
		end

		elseif messageid==0x4404 then id_str="ReportVelocityState"
		local vx_pose        =  17
		local const low_vx   = -327.68
		local const high_vx  =  327.67
		local const low_yaw  = -32.768
		local const high_yaw =  32.767


		local scale_vx         = buffer(vx_pose,int_l):le_uint()
		vx = UInt32Toscale(scale_vx,low_vx,high_vx)

		local scale_yaw_rate   = buffer(vx_pose+int_l,short_l):le_uint()
		yaw_rate = UInt16Toscale(scale_yaw_rate,low_yaw,high_yaw)

		local Time_stamp = buffer(vx_pose+int_l+short_l,int_l):le_uint()
		Time = Timestamp2time(Time_stamp)

		subtree:add_le(buffer(vx_pose,int_l),"vx: " .. string.format('%x',vx))

		subtree:add_le(buffer(vx_pose+int_l,short_l),"yaw_rate: " .. string.format('%x',yaw_rate))

		subtree:add_le(buffer(vx_pose+int_l+short_l,int_l),"Time Stamp = Day: " .. string.format('%i',Time[1]).." Time: " .. string.format('%02i',Time[2]) ..":" .. string.format('%02i',Time[3])..":" .. string.format('%02i',Time[4]).."." .. string.format('%03i',Time[5]))

		elseif messageid==0x4405 then id_str="ReportWrenchEffort"
		elseif messageid==0x4407 then id_str="ReportGlobalVector"
		elseif messageid==0x4408 then id_str="ReportLocalVector"
		elseif messageid==0x440A then id_str="ReportTravelSpeed"
		local scale_speed = buffer(15,2):le_uint()
		speed = UInt16Toscale(scale_speed,lower_sl,upper_sl)
		subtree:add_le(buffer(15,short_l),"speed: " .. string.format('%s',speed))

		elseif messageid==0x440C then id_str="ReportGlobalWaypoint"
		elseif messageid==0x440D then id_str="ReportLocalWaypoint" --Y raw data wrong

		local xw_pose  = 16
		local scale_xw = buffer(xw_pose,int_l):le_uint()
		xw  = UInt32Toscale(scale_xw,low_x_y_z,high_x_y_z)

		local scale_yw = buffer(xw_pose+int_l,int_l):le_uint()
		yw  = UInt32Toscale(scale_yw,low_x_y_z,high_x_y_z)

		subtree:add_le(buffer(xw_pose,int_l),"xw:   " .. string.format('%8.2f',xw))

		subtree:add_le(buffer(xw_pose+int_l,int_l),"yw:   " .. string.format('%8.2f',yw))

		elseif messageid==0x440F then id_str="ReportGlobalPathSegment"
		elseif messageid==0x4410 then id_str="ReportLocalPathSegment"
		elseif messageid==0x4412 then id_str="ReportGeomagneticProperty"
		elseif messageid==0x4415 then id_str="ReportVelocityCommand"
		elseif messageid==0x4416 then id_str="ReportAccelerationLimit"
		elseif messageid==0x4417 then id_str="ReportAccelerationState"
		elseif messageid==0x441A then id_str="ReportElement"
		elseif messageid==0x441B then id_str="ReportElementList"

		local Max_ELEMENT_Numbers = 4
		local count_field_pose = 15
		local count_field = buffer(count_field_pose,short_l):le_uint()
		local UID = {}
		local UID_pose = {}

		subtree:add_le(buffer(count_field_pose,short_l),"count_field:   " .. string.format('%x',count_field))

		for i=1, Max_ELEMENT_Numbers do if i == 1 then
				UID_pose[i] = count_field_pose+short_l
				UID[i]   = buffer(UID_pose[i],short_l):le_uint()
			else
				UID_pose[i] = UID_pose[i-1] + short_l
				UID[i]   = buffer(UID_pose[i],short_l):le_uint()
			end
			subtree:add_le(buffer(UID_pose[i],short_l),"UID [" .. string.format('%i',i).."] information:" .. string.format('%s',UID[i]))
		end

		elseif messageid==0x441C then id_str="ReportElementCount"
		local count = buffer(15,2):le_uint()
		subtree:add_le(buffer(15,2),"count:   " .. string.format('%x',count))

		elseif messageid==0x441E then id_str="ReportActiveElement"
		local active_id = buffer(15,2):le_uint()
		subtree:add_le(buffer(15,2),"active_id:   " .. string.format('%x',active_id))

	end

end

local offset = 1
--subtree:add (f.messagetype,   buffer(offset, 1))
--subtree:add (f.datasize,      buffer(offset, 2))
--subtree:add (f.pbad ,         buffer(offset, 1))
--subtree:add (f.destinationid, buffer(offset, 4))
--subtree:add (f.sourceid ,     buffer(offset, 4))
--subtree:add (f.payload,       buffer(offset, 3))
--local messageid=buffer(13,2):le_uint()
--subtree:add (f.messageid,messageid)
--subtree:add_le (f.seq_number,    buffer(offset, 2))



end
-- end of function jaus_proto.dissector
 DissectorTable.get("udp.port"):add(3794, jaus_proto)
 info(jaus_proto.name)
end



--Convert bytes array to string
function bytestostring2 (bytes,len)
	local w = {}
	for i=1,len do w[i] = string.char(bytes[i]:le_uint()) end
	return table.concat(w)
end

--Define some helper functions to convert to/from scaled integers
function UInt32Toscale(val, low, high)
	local scale = ((val  *(high - low)) / UInt32Range) + low
	return scale
end

function UInt16Toscale(val, low, high)
	local scale = ((val  *(high - low)) / UInt16Range) + low
		return scale
end

function Timestamp2time (Time_stamp)
	local day_mask       =   0xF8000000
	local hour_mask      =   0x07C00000
	local minutes_mask   =   0x003F0000
	local seconds_mask   =   0x0000FC00
	local ms_mask        =   0x000003FF
	Time = {}
	Time[1] = bit.rshift(bit.band(Time_stamp, day_mask),27)
	Time[2] = bit.rshift(bit.band(Time_stamp, hour_mask),22)
	Time[3] = bit.rshift(bit.band(Time_stamp, minutes_mask),16)
	Time[4] = bit.rshift(bit.band(Time_stamp, seconds_mask),10)
	Time[5] = bit.band(Time_stamp, ms_mask)

	return Time

end

