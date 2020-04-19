extends Camera

var _previousPosition: Vector2 = Vector2(0, 0);
var _moveCamera: bool = false;
var _motion_vector: Vector2 = Vector2(0, 0);

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

func _unhandled_input(event):
	if event is InputEventMouseButton && event.button_index == BUTTON_LEFT:
		get_tree().set_input_as_handled();
		if event.is_pressed():
			_previousPosition = event.position;
			_moveCamera = true;
		else:
			_moveCamera = false;	
	elif event is InputEventMouseMotion && _moveCamera:
		get_tree().set_input_as_handled();
		_motion_vector = _previousPosition - event.position;
