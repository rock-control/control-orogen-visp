require 'pry'
require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'


describe 'visp::Task checking' do
  include Orocos::Test::Component
  start 'task', 'visp::Task' => 'task'
  reader 'task', 'cmd_out', :attr_name => 'cmd_out'
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
    aux = Types::Apriltags::VisualFeaturePoint.new
    aux.time = Time.now
    aux.identifier = "apriltags_20"
    for i in 0...4
      aux.points << Types::Base::Vector2d.new
    end
    aux.points[0]["data"] = p1 
    aux.points[1]["data"] = p2
    aux.points[2]["data"] = p3
    aux.points[3]["data"] = p4
    corners_samples << aux
    corners_samples
  end

  #it 'no camera parameters' do
  #it 'object on the first quadrant' do

  #it 'if there is no visible markers' do
  #  task.apply_conf_file("visp::Task.yml")

  #  task.configure
  #  task.start

  #  assert_state_change(task) {|state| state == :WAITING_CORNERS}
  #end


  it 'if no corners are informed' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start
    
    #writing the setpoint
    setpoint_samples = create_setpoint(1, 1, 1, 0, 0, 0)

    sleep(0.01)
    cmd_in.write setpoint_samples
    assert((task.state_reader.read == :WAITING_CORNERS), "the state was supposed to be
    :WAITING_CORNERS, however it is #{task.state_reader.read}")
  end

  it 'if no setpoint is informed' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])

    marker_corners.write corners_samples
    sleep(0.01)
    assert((task.state_reader.read == :WAITING_SETPOINT), "the state was supposed to be
    :WAITING_SETPOINT, however it is #{task.state_reader.read}")
  end

  it 'if there is corners and setpoint' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])
    #creating setpoint
    setpoint_samples = create_setpoint(1, 1, 1, 0, 0, 0)

    marker_corners.write corners_samples
    cmd_in.write setpoint_samples
    sleep(0.01)
    assert((task.state_reader.read == :CONTROLLING), "the state was supposed to be
    :CONTROLLING, however it is #{task.state_reader.read}")

  end

  it 'if multiple state changes works as expected' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start
    #creating the corner_samples
    corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])
    #creating setpoint
    setpoint_samples = create_setpoint(1, 1, 1, 0, 0, 0)

    sleep(0.01)
    assert((task.state_reader.read == :WAITING_CORNERS), "the state was supposed to be
    :WAITING_CORNERS, however it is #{task.state_reader.read}")

    marker_corners.write corners_samples
    sleep(0.01)
    assert((task.state_reader.read == :WAITING_SETPOINT), "the state was supposed to be
    :WAITING_SETPOINT, however it is #{task.state_reader.read}")

    marker_corners.write corners_samples
    cmd_in.write setpoint_samples
    sleep(0.01)
    assert((task.state_reader.read == :CONTROLLING), "the state was supposed to be
    :CONTROLLING, however it is #{task.state_reader.read}")

  end

  it 'if visual feature are in the first quadrant' do
    task.apply_conf_file("visp::Task.yml")
    task.configure
    task.start

    #creating the corner_samples
    corners_samples = create_corners([1400, 600], [1400, 400], [1600, 400], [1600, 600])
    #creating setpoint
    setpoint_samples = create_setpoint(0, 0, 1, 0, 0, 0)

    marker_corners.write corners_samples
    cmd_in.write setpoint_samples
    sleep(0.01)
    data = assert_has_one_new_sample cmd_out,1 

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

    marker_corners.write corners_samples
    cmd_in.write setpoint_samples
    sleep(0.01)
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

    marker_corners.write corners_samples
    cmd_in.write setpoint_samples
    sleep(0.01)
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

    marker_corners.write corners_samples
    cmd_in.write setpoint_samples
    sleep(0.01)
    data = assert_has_one_new_sample cmd_out,1 

    assert((data.linear[0] > 0) && (data.linear[1] > 0) && (data.linear[2] > 0), 
      "the result is not the expected when the object is on the third quadrant, values
      are #{data.linear[0]}, #{data.linear[1]}, #{data.linear[2]}")
  end

end


