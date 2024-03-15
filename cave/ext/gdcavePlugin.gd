@tool
extends EditorPlugin


func _enter_tree():
	add_custom_type("GDCave", "Object", preload("gdcave.gd"),preload("res://icon.svg"))
	pass


func _exit_tree():
	remove_custom_type("GDCave")
