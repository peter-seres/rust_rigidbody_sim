extends Spatial

# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#	pass

func _process(delta):
	if Input.is_action_just_pressed("ui_accept"):
		if $InterpolatedCamera.is_current():
			$StaticCamera.make_current()
		else:
			$InterpolatedCamera.make_current()
