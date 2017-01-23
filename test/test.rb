require 'pry-byebug'
require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'
require "transformer/runtime"

describe 'visp::Task checking' do
  include Orocos::Test::Component
  start 'task', 'visp::Task' => 'task'
  reader 'task', 'cmd_out', :attr_name => 'cmd_out'
  reader 'task', 'controller_state', :attr_name => 'controller_state'
  writer 'task', 'cmd_in', :attr_name => 'cmd_in'
  writer 'task', 'marker_corners', :attr_name => 'marker_corners'

  def create_setpoint(x, y, z, roll, pitch, yaw)
    setpoint = task.cmd_in.new_sample
    setpoint.linear[0] = x 
    setpoint.linear[1] = y
    setpoint.linear[2] = z
    setpoint.angular[0] = roll
    setpoint.angular[1] = pitch
    setpoint.angular[2] = yaw
    setpoint
  end

  def create_corners(p1, p2, p3, p4)
    #writing the markers
    corners_samples = task.marker_corners.new_sample
    aux = Types.apriltags.VisualFeaturePoint.new
    aux.time = Time.now
    aux.identifier = "apriltags_20"
    for i in 0...4
      aux.points << Types.base.Vector2d.new
    end
    aux.points[0]["data"] = p1 
    aux.points[1]["data"] = p2
    aux.points[2]["data"] = p3
    aux.points[3]["data"] = p4
    corners_samples << aux
    corners_samples
  end

  def assert_task_state(task, expected_state)
      task_state = task.state_reader.read
      sleep(0.01)
      assert((task_state == expected_state), "it is expected #{expected_state} 
             state, however it is receiving #{task_state}")
  end



  #it 'if no corners are informed' do
  #  task.apply_conf_file("visp::Task.yml")
  #  task.configure
  #  task.start
  #  
  #  #writing the setpoint
  #  setpoint_samples = create_setpoint(1, 1, 1, 0, 0, 0)
  #  cmd_in.write setpoint_samples

  #  assert_task_state(task, :WAITING_CORNERS)
  #end

  #it 'if no setpoint is informed' do
  #  task.apply_conf_file("visp::Task.yml")
  #  task.configure
  #  task.start

  #  #writing corners
  #  corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])
  #  marker_corners.write corners_samples
  #  assert_task_state(task, :WAITING_SETPOINT)
  #end

  #it 'if there is corners and setpoint' do
  #  task.apply_conf_file("visp::Task.yml")
  #  task.configure
  #  task.start

  #  corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])
  #  setpoint_samples = create_setpoint(1, 1, 1, 0, 0, 0)

  #  #writing ports
  #  #NOTE: Since the component is triggered by the marker_corners input port
  #  #it has to be written after the setpoint
  #  cmd_in.write setpoint_samples
  #  marker_corners.write corners_samples
  #  assert_task_state(task, :CONTROLLING)
  #end

  #it 'if multiple state changes works as expected' do
  #  task.apply_conf_file("visp::Task.yml")
  #  task.configure
  #  task.start

  #  corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])
  #  setpoint_samples = create_setpoint(1, 1, 1, 0, 0, 0)

  #  assert_task_state(task, :WAITING_CORNERS)

  #  marker_corners.write corners_samples
  #  assert_task_state(task, :WAITING_SETPOINT)

  #  #NOTE: Since the component is triggered by the marker_corners input port
  #  #it has to be written after the setpoint
  #  cmd_in.write setpoint_samples
  #  marker_corners.write corners_samples
  #  assert_task_state(task, :CONTROLLING)

  #end

  it 'if visual feature are in the first quadrant' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])
    #creating setpoint
    setpoint_samples = create_setpoint(0, 0, 1, 0, 0, 0)

    #NOTE: Since the component is triggered by the marker_corners input port
    #it has to be written after the setpoint
    cmd_in.write setpoint_samples
    marker_corners.write corners_samples
    data = assert_has_one_new_sample cmd_out,5 

    assert((data.linear[0] > 0) && (data.linear[1] < 0) && (data.linear[2] > 0), 
      "the result is not the expected when the object is on the first quadrant
      are #{data.linear[0]}, #{data.linear[1]}, #{data.linear[2]}")
  end

  it 'if visual feature are on the second quadrant' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([400, 600], [400, 400], [600, 400], [600, 600])
    #creating setpoint
    setpoint_samples = create_setpoint(0, 0, 1, 0, 0, 0)

    #NOTE: Since the component is triggered by the marker_corners input port
    #it has to be written after the setpoint
    cmd_in.write setpoint_samples
    marker_corners.write corners_samples
    data = assert_has_one_new_sample cmd_out,1 

    assert((data.linear[0] < 0) && (data.linear[1] < 0) && (data.linear[2] > 0), 
      "the result is not the expected when the object is on the third quadrant, values
      are #{data.linear[0]}, #{data.linear[1]}, #{data.linear[2]}")
  end

  it 'if visual feature are on the third quadrant' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([400, 1600], [400, 1400], [600, 1400], [600, 1600])
    #creating setpoint
    setpoint_samples = create_setpoint(0, 0, 1, 0, 0, 0)

    #NOTE: Since the component is triggered by the marker_corners input port
    #it has to be written after the setpoint
    cmd_in.write setpoint_samples
    marker_corners.write corners_samples
    data = assert_has_one_new_sample cmd_out,1 

    assert((data.linear[0] < 0) && (data.linear[1] > 0) && (data.linear[2] > 0), 
      "the result is not the expected when the object is on the third quadrant, values
      are #{data.linear[0]}, #{data.linear[1]}, #{data.linear[2]}")
  end

  it 'if visual feature are on the fourth quadrant' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([1400, 1600], [1400, 1400], [1600, 1400], [1600, 1600])
    #creating setpoint
    setpoint_samples = create_setpoint(0, 0, 1, 0, 0, 0)

    #NOTE: Since the component is triggered by the marker_corners input port
    #it has to be written after the setpoint
    cmd_in.write setpoint_samples
    marker_corners.write corners_samples
    data = assert_has_one_new_sample cmd_out,1 

    assert((data.linear[0] > 0) && (data.linear[1] > 0) && (data.linear[2] > 0), 
      "the result is not the expected when the object is on the third quadrant, values
      are #{data.linear[0]}, #{data.linear[1]}, #{data.linear[2]}")
  end
  
  it 'if it apply the transformation correctly' do
    Orocos.conf.load_dir('.')
    Orocos.conf.apply(task,['default'])

    Orocos.transformer.load_conf("static_transforms.rb")
    Orocos.transformer.setup(task)

    #increase the gain, so the differences is better noticed
    task.gain = 0.2
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([400, 1600], [400, 1400], [600, 1400], [600, 1600])
    #creating setpoint
    ang30 = 30*3.1417/180
    ang90 = 90*3.1417/180
    setpoint_samples = create_setpoint(2, 0, 0, 0, 0, -ang90)

    #NOTE: Since the component is triggered by the marker_corners input port
    #it has to be written after the setpoint
    cmd_in.write setpoint_samples
    marker_corners.write corners_samples
    #data_base = assert_has_one_new_sample cmd_out, 1
    data_base = assert_has_one_new_sample controller_state, 1
    vel_base = assert_has_one_new_sample cmd_out, 1
    
    #write  a setpoint of 30 degress to yaw.
    setpoint_samples = create_setpoint(2, 0, 0, 0, 0, -ang90+ang30)
    cmd_in.write setpoint_samples
    marker_corners.write corners_samples
    #data = assert_has_one_new_sample cmd_out, 1
    data = assert_has_one_new_sample controller_state, 1
    vel = assert_has_one_new_sample cmd_out, 1

    puts "vel_base.angular #{vel_base.angular[2]}"
    puts "vel.angular #{vel.angular[2]}"
    
    puts "data_base.angular #{data_base.desired_pose.orientation.roll}"
    puts "data_base.angular #{data_base.desired_pose.orientation.pitch}"
    puts "data_base.angular #{data_base.desired_pose.orientation.yaw}"

    puts "data.angular #{data.desired_pose.orientation.roll}"
    puts "data.angular #{data.desired_pose.orientation.pitch}"
    puts "data.angular #{data.desired_pose.orientation.yaw}"

    assert((data.angular[2] < 0))
    assert((data_base.angular[2] - data.angular[2]) > 0.01)
  end
end
