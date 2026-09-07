# Evaluate Quartus source paths without requiring Quartus.
set quartus(version) "24.1"
set quartus(qip_path) [pwd]
set records {}
proc set_global_assignment {args} {
 global records quartus
 set i [lsearch -exact $args -name]
 set key [lindex $args [expr {$i+1}]]
 set value [lindex $args [expr {$i+2}]]
 lappend records [list $key $value]
 if {$key eq "QIP_FILE"} {
  set save $quartus(qip_path)
  set quartus(qip_path) [file dirname [file normalize $value]]
  source $value
  set quartus(qip_path) $save
 }
}
proc set_instance_assignment {args} {}
proc set_location_assignment {args} {}
proc unknown {cmd args} {
 if {[regexp {^[0-9*]+$} $cmd]} {return "\[$cmd\]"}
 error "unknown Tcl command $cmd"
}
source HardDrivin.qsf
foreach r $records {puts "[lindex $r 0]\t[lindex $r 1]"}
