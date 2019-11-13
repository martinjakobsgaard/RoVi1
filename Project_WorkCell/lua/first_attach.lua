wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
bottle = wc:findFrame("Bottle")
gripper = wc:findFrame("GraspTCP")

function attach(obj, tool)
rw.gripFrame(obj, tool, state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

attach(bottle, gripper)