//
//  NNHDMapViewController.swift
//  NNMintFurniture
//
//  Created by xiaoliang.ren on 2018/10/10.
//  Copyright © 2018年 zhongNing. All rights reserved.
//

import UIKit
import SceneKit
import SpriteKit
import GameplayKit
import Alamofire
import SwiftyJSON
import WebSocket

public class NNHDMapViewController : UIViewController {
    var lastX = 0.0, lastY = 0.0, lastZ = 60.0, lastScale = 1.0, lastRot: Double = 0.0
    var scnView: SCNView!
    var scnScene: SCNScene!
    var auto_move = 2

    convenience init() {
        self.init(nibName:nil, bundle:nil)
        self.lastX = 0.0
        self.lastY = 0.0
        self.lastZ = 60.0
        self.lastScale = 1.0
        self.lastRot = 0.0
        self.view = SCNView()
        viewDidLoad()
        
        //创建定时任务
        _ = Timer.scheduledTimer(timeInterval: 0.1,target:self,selector:#selector(NNHDMapViewController.timerFireMethod),
                                 userInfo:nil,repeats:true)
    }
    // 参数: The timer passes itself as the argument
    func timerFireMethod() {
        
        let headers: HTTPHeaders = [
            "Host" : "ihome.ngrok.xiaomiqiu.cn",
            "Connection" : "keep-alive",
            "Authorization" : "Basic dXNlcjoxMjM0NQ==",
            "Accept": "application/json"
        ]
        Alamofire.request("http://ihome.ngrok.xiaomiqiu.cn/asp/car_info.asp",method:.get, parameters: ["foo": "bar"],headers:headers)
            .response { response in
                //print("Request: \(response.request)")
                //print("Response: \(response.response)")
                //print("Error: \(response.error)")
                if let data = response.data, let _ = String(data: data, encoding: .utf8) {
                    //print("Data: \(data)")
                    //转成JSON对象
                    var jsondata = JSON(response.data ?? data)
                    //print("status :\(jsondata["status"].stringValue)")
                    //print("car_id ::\(jsondata["array"][0]["car_id"].stringValue)")
                    //var car_list = jsondata[]
                    self.addcarScreenAnnotation(jsondata: jsondata)
                }
        }
    }
    //添加固定car屏幕位置的标注
    func addcarScreenAnnotation(jsondata: JSON) {
        
        let car_list = jsondata["array"].arrayValue
        for car in car_list{
            //print("\(car["car_id"].stringValue)")
            //print("update car pos:\(Float(car["x"].doubleValue), Float(car["y"].doubleValue),Float(car["yaw"].doubleValue))")
            updateCar(SCNVector3Make(Float(car["x"].doubleValue), Float(car["y"].doubleValue),Float(car["yaw"].doubleValue)))
            if(auto_move == 2){
                set_view_point(xx: Float(car["x"].doubleValue), yy: Float(car["y"].doubleValue))
            }
        }
    }
    
    override public var shouldAutorotate: Bool {
        return true
    }
    override public var prefersStatusBarHidden: Bool {
        return true
    }
    
    func setupView() {
        scnView = self.view as? SCNView
    }
    
    func setupScene() {
        scnScene = SCNScene()
        scnView.scene = scnScene
    }
    @objc func tapHandler(_ gesture: UITapGestureRecognizer) {
        lastZ = 60.0
        scnView.pointOfView?.eulerAngles.x = 0.0
        scnView.pointOfView?.eulerAngles.y = 0.0
        scnView.pointOfView?.eulerAngles.z = 0.0
        scnView.pointOfView?.position.z = Float(lastZ)
    }
    
    @objc func panHandler(_ gesture: UIPanGestureRecognizer) {
        let numberOfTouches = gesture.numberOfTouches
        let translation = gesture.translation(in: gesture.view)
        
        if (gesture.state == UIGestureRecognizerState.changed) {
            let dx = Double(translation.x) - lastX
            let dy = Double(translation.y) - lastY
            lastX = Double(translation.x)
            lastY = Double(translation.y)
            if (numberOfTouches == 1) {
                let ndx: Double = -0.00285*dx*lastZ
                let ndy: Double = 0.00285*dy*lastZ
                
                let theta: Double = Double(scnView.pointOfView!.eulerAngles.z)
                scnView.pointOfView?.position.x += Float(ndx * cos(theta) - ndy * sin(theta))
                scnView.pointOfView?.position.y += Float(ndy * cos(theta) + ndx * sin(theta))
                //updateCar(SCNVector3Make(scnView.pointOfView!.position.x, scnView.pointOfView!.position.y, scnView.pointOfView!.position.x / 100.0))
                auto_move = 1
            }
            else if (numberOfTouches == 2) {
                let dRoll = 0.00285 * dy
                let dPitch = 0.00285 * dx
                
                var newRoll = scnView.pointOfView!.eulerAngles.x + Float(dRoll)
                let maxAngle: Float = 1.0
                if newRoll > maxAngle {newRoll = maxAngle}
                if newRoll < -maxAngle {newRoll = -maxAngle}
                var newPitch = scnView.pointOfView!.eulerAngles.y + Float(dPitch)
                if newPitch > maxAngle {newPitch = maxAngle}
                if newPitch < -maxAngle {newPitch = -maxAngle}
                scnView.pointOfView?.eulerAngles.x = newRoll
                scnView.pointOfView?.eulerAngles.y = newPitch
                auto_move = 2
            }
        }
        else if (gesture.state == UIGestureRecognizerState.ended) {
            lastX = 0.0
            lastY = 0.0
        }
    }
    
    @objc func pinchHandler(_ gesture: UIPinchGestureRecognizer) {
        let numberOfTouches = gesture.numberOfTouches
        let scale: Double = Double(gesture.scale)
        
        if (gesture.state == UIGestureRecognizerState.changed) {
            if (numberOfTouches == 2) {
                let dScale = scale / lastScale
                lastScale = scale
                lastZ = lastZ / dScale
                if lastZ > 500 {lastZ = 500}
                if lastZ < 2 {lastZ = 2}
                
                scnView.pointOfView?.position.z = Float(lastZ)
                
            }
        }
        else if (gesture.state == UIGestureRecognizerState.ended) {
            lastScale = 1.0
        }
    }
    
    @objc func rotationHandler(_ gesture: UIRotationGestureRecognizer) {
        let numberOfTouches = gesture.numberOfTouches
        let rot: Double = Double(gesture.rotation)
        
        if (gesture.state == UIGestureRecognizerState.changed) {
            if (numberOfTouches == 2) {
                let dRot = rot - lastRot
                lastRot = rot
                //if let view = self.view as! SCNView? {
                let newRot = scnView.pointOfView!.eulerAngles.z + Float(dRot)
                scnView.pointOfView?.eulerAngles.z = newRot
                //}
            }
        }
        else if (gesture.state == UIGestureRecognizerState.ended) {
            lastRot = 0.0
        }
    }
    
    func updateCar(_ position: SCNVector3) {
        // if let view = self.view as! SCNView? {
        let carNode = scnView.scene?.rootNode.childNode(withName: "car_body", recursively: false)
        let yaw = position.z
        carNode?.position = SCNVector3Make(position.x - 1.4 * cos(yaw), position.y - 1.4 * sin(yaw), 0)
        carNode?.eulerAngles = SCNVector3(1.5708, 0, yaw + 3.1416)
        let wheelNodeFL = scnView.scene?.rootNode.childNode(withName: "car_wheel_fl", recursively: false)
        wheelNodeFL?.position = SCNVector3Make(
            position.x + 1.4 * cos(yaw) - 0.8 * sin(yaw),
            position.y + 1.4 * sin(yaw) + 0.8 * cos(yaw),
            0.0)
        wheelNodeFL?.eulerAngles = SCNVector3(0, 0, yaw)
        let wheelNodeFR = scnView.scene?.rootNode.childNode(withName: "car_wheel_fr", recursively: false)
        wheelNodeFR?.position = SCNVector3Make(
            position.x + 1.4 * cos(yaw) + 0.8 * sin(yaw),
            position.y + 1.4 * sin(yaw) - 0.8 * cos(yaw),
            0.0)
        wheelNodeFR?.eulerAngles = SCNVector3(0, 0, yaw)
        let wheelNodeRL = scnView.scene?.rootNode.childNode(withName: "car_wheel_rl", recursively: false)
        wheelNodeRL?.position = SCNVector3Make(
            position.x - 1.4 * cos(yaw) - 0.8 * sin(yaw),
            position.y - 1.4 * sin(yaw) + 0.8 * cos(yaw),
            0.0)
        wheelNodeRL?.eulerAngles = SCNVector3(0, 0, yaw)
        let wheelNodeRR = scnView.scene?.rootNode.childNode(withName: "car_wheel_rr", recursively: false)
        wheelNodeRR?.position = SCNVector3Make(
            position.x - 1.4 * cos(yaw) + 0.8 * sin(yaw),
            position.y - 1.4 * sin(yaw) - 0.8 * cos(yaw),
            0.0)
        wheelNodeRR?.eulerAngles = SCNVector3(0, 0, yaw)
        // }
        let route : [String] = ["1", "2", "3", "5", "8"]
        if Int(arc4random()) % 100 == 0 {
            updateMapWithRoute(route: route)
        }
    }
    
    public override func viewDidLoad() {
        super.viewDidLoad()
        setupView()
        let camera = SCNCamera()
        camera.fieldOfView = 90
        camera.zFar = 2000
        
        guard let scene = SCNScene(named : "scene.scnassets/scene.scn")
            else {
                print("unable to get target scn")
                return
        }
        
        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(self.tapHandler(_:)))
        tapGesture.numberOfTapsRequired = 2
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(self.panHandler(_:)))
        let pinchGesture = UIPinchGestureRecognizer(target: self, action: #selector(self.pinchHandler(_:)))
        let rotationGesture = UIRotationGestureRecognizer(target: self, action: #selector(self.rotationHandler(_:)))
        
        if let scnView = self.view as! SCNView? {
            scnView.scene = scene
            scnView.allowsCameraControl = true
            scnView.pointOfView?.camera = camera
            scnView.pointOfView?.position.z = Float(lastZ)
            scnView.pointOfView?.eulerAngles.x = 0
            scnView.addGestureRecognizer(tapGesture)
            scnView.addGestureRecognizer(panGesture)
            scnView.addGestureRecognizer(pinchGesture)
            scnView.addGestureRecognizer(rotationGesture)
            scnView.pointOfView?.position.x = -500
            scnView.pointOfView?.position.y = -550
        }
        
        
        
        //        guard let scene = SCNScene(named : "scene.scnassets/scene.scn")
        //            else {
        //                print("unable to get target scn")
        //                return
        //        }
        //
        //        if let view = self.view as! SCNView? {
        //            view.scene = scene
        //            view.allowsCameraControl = true
        //        }
        
        
        let carScene = SCNScene(named: "scene.scnassets/carbody_visual.dae")
        let carNode = carScene!.rootNode.childNodes[0]
        carNode.name = "car_body"
        scene.rootNode.addChildNode(carNode)
        
        let wheelSceneFL = SCNScene(named: "scene.scnassets/wheel_visual.dae")
        let wheelNodeFL = wheelSceneFL!.rootNode.childNodes[0]
        wheelNodeFL.name = "car_wheel_fl"
        scene.rootNode.addChildNode(wheelNodeFL)
        
        let wheelSceneFR = SCNScene(named: "scene.scnassets/wheel_visual.dae")
        let wheelNodeFR = wheelSceneFR!.rootNode.childNodes[0]
        wheelNodeFR.name = "car_wheel_fr"
        scene.rootNode.addChildNode(wheelNodeFR)
        
        let wheelSceneRL = SCNScene(named: "scene.scnassets/wheel_visual.dae")
        let wheelNodeRL = wheelSceneRL!.rootNode.childNodes[0]
        wheelNodeRL.name = "car_wheel_rl"
        scene.rootNode.addChildNode(wheelNodeRL)
        
        let wheelSceneRR = SCNScene(named: "scene.scnassets/wheel_visual.dae")
        let wheelNodeRR = wheelSceneRR!.rootNode.childNodes[0]
        wheelNodeRR.name = "car_wheel_rr"
        scene.rootNode.addChildNode(wheelNodeRR)
        
        //
        //        let nodearr = daescene?.rootNode.childNodes
        //
        //        for child in nodearr! {
        //            child.position.x = 0
        //            child.eulerAngles.x = 0
        //            child.eulerAngles.y = 0
        //            child.eulerAngles.z = 0
        //
        //
        //            scene.rootNode.addChildNode(child)
        //
        //        }
        //
        //
        //        let daescene2 = SCNScene(named: "scene.scnassets/wheel_visual.dae")
        //
        //        let nodearr2 = daescene2?.rootNode.childNodes
        //
        //        for child in nodearr2! {
        //            child.position.x = 0
        //            child.eulerAngles.x = 0
        //            child.eulerAngles.y = 0
        //            child.eulerAngles.z = 0
        //
        //
        //            scene.rootNode.addChildNode(child)
        //
        //        }
        
        let filepath = Bundle.main.path(forResource: "zgc", ofType: "xodr")
        SetFile(filepath)
        
        let map = GetMap()
        
        let map_node = SCNNode(geometry: nil)
        map_node.name = "map"
        
        for lane_node in drawMap(map : map) {
            map_node.addChildNode(lane_node)
        }
        let objects = GetRoadObjects()
        for object_node in drawRoadObjects(objects: objects) {
            map_node.addChildNode(object_node)
        }
        scene.rootNode.addChildNode(map_node)
        
    }
    func set_view_point(xx:Float,yy:Float){
        scnView.pointOfView?.position.x = xx
        scnView.pointOfView?.position.y = yy
    }
    public func updateMapWithRoute(route : [String]) {
        if let view = self.view as! SCNView? {
            view.scene?.rootNode.childNode(withName: "map", recursively: false)?.removeFromParentNode()
            let map_node = SCNNode(geometry: nil)
            map_node.name = "map"
            let map = GetMap()
            
            for lane_node in drawMap(map : map) {
                map_node.addChildNode(lane_node)
            }
            for route_node in drawRoute(map : map, route : route) {
                map_node.addChildNode(route_node)
            }
            let objects = GetRoadObjects()
            for object_node in drawRoadObjects(objects: objects) {
                map_node.addChildNode(object_node)
            }
            view.scene?.rootNode.addChildNode(map_node)
        }
    }
    
    public func drawMap(map : CMap) -> [SCNNode] {
        
        var result = [SCNNode]()
        
        for road_index in Array(0 ..< map.roads_size) {
            let road = map.roads[Int(road_index)]
            for sec_index in Array(0 ..< road.laneSections_size) {
                let section = road.laneSections[Int(sec_index)]
                for lane_index in Array(0 ..< section.lanes_size) {
                    let lane = section.lanes[Int(lane_index)]
                    if lane_index > 0 {
                        let lane_l = section.lanes[Int(lane_index - 1)]
                        var vertices = [SCNVector3]()
                        for p_index in Array(0 ..< lane.points_size) {
                            if (p_index + 1 < lane.points_size){
                                vertices += drawFace([lane_l.points[Int(p_index)], lane.points[Int(p_index)], lane_l.points[Int(p_index + 1)], lane.points[Int(p_index + 1)]])
                            }
                        }
                        result += [getTranglesNode(vertices, color: UIColor.darkGray.withAlphaComponent(1.0))]
                    }
                    let markNode = getRoadMarkNode(lane)
                    if markNode != nil {result += [markNode!]}
                }
            }
        }
        
        return result
    }
    
    public func drawRoute(map : CMap, route : [String]) -> [SCNNode] {
        
        var result = [SCNNode]()
        
        for road_index in Array(0 ..< map.roads_size) {
            let road = map.roads[Int(road_index)]
            if !route.contains(String(cString: road.id)) {continue}
            for sec_index in Array(0 ..< road.laneSections_size) {
                let section = road.laneSections[Int(sec_index)]
                for lane_index in Array(0 ..< section.lanes_size) {
                    let lane = section.lanes[Int(lane_index)]
                    if lane.id == 0 {
                        let markNode = getRouteMarkNode(lane)
                        if markNode != nil {result += [markNode!]}
                    }
                }
            }
        }
        
        return result
    }
    
    func getRoadMarkNode(_ lane : CLane) -> SCNNode? {
        if lane.roadMarks_size == 0 || lane.points_size < 2 {return nil}
        let markColor = String(cString: lane.roadMarks[0].color)
        let roadMarks = lane.roadMarks!
        var index: Int = 0
        var half_width = 0.075
        let base_s = roadMarks[0].sOffset + lane.points[0].s
        var next_s = base_s
        var last_p = lane.points[0]
        var p = last_p
        var t = 0.0
        var l = 0
        
        var vertices = [SCNVector3]()
        while l < lane.points_size-1 {
            if index >= lane.roadMarks_size {break}
            if roadMarks[index].width > 0 {half_width = roadMarks[index].width / 2.0}
            let new_s = index + 1 < lane.roadMarks_size ? roadMarks[index + 1].sOffset + base_s : lane.points[Int(lane.points_size) - 1].s
            let dist = new_s - next_s
            if String(cString: roadMarks[index].type) == "none" {
                last_p = p
                (p, l, t) = getNextPoint(lane, l, t, new_s)
            }
            else if String(cString: roadMarks[index].type) == "solid" {
                last_p = p
                let old_l = l
                (p, l, t) = getNextPoint(lane, l, t, new_s)
                if old_l+1 <= l {
                    for k in old_l+1 ... l {
                        vertices += getRectPoints(last_p, lane.points[k], half_width)
                        last_p = lane.points[k]
                    }
                }
                vertices += getRectPoints(last_p, p, half_width)
            }
            else if String(cString: roadMarks[index].type) == "broken" {
                var repeat_num: Double = Double(Int(dist / 10.0 + 0.5))
                if repeat_num < 1 {repeat_num = 1}
                let full_length: Double = dist / repeat_num * 0.4
                //                let empty_length: Double = dist / repeat_num * 0.6
                let segment_length: Double = dist / repeat_num
                var c: Double = 0.0
                while p.s - new_s < -0.1 && l < lane.points_size - 1 {
                    last_p = p
                    let old_l = l
                    (p, l, t) = getNextPoint(lane, l, t, next_s + c * segment_length + full_length)
                    c += 1.0
                    if old_l+1 <= l {
                        for k in old_l+1 ... l {
                            vertices += getRectPoints(last_p, lane.points[k], half_width)
                            last_p = lane.points[k]
                        }
                    }
                    vertices += getRectPoints(last_p, p, half_width)
                    (p, l, t) = getNextPoint(lane, l, t, next_s + c * segment_length)
                }
            }
            next_s = new_s
            index += 1
        }
        
        if markColor == "yellow" {
            return getTranglesNode(vertices, color: UIColor.yellow.withAlphaComponent(1.0))
        }
        else {
            return getTranglesNode(vertices, color: UIColor.white.withAlphaComponent(1.0))
        }
    }
    
    func getRouteMarkNode(_ lane : CLane) -> SCNNode? {
        if lane.roadMarks_size == 0 || lane.points_size < 2 {return nil}
        let half_width = 0.25
        var l = 0
        
        var vertices = [SCNVector3]()
        while l < lane.points_size-1 {
            vertices += getRectPoints(lane.points[l], lane.points[l + 1], half_width, 0.1)
            l += 1
        }
        
        return getTranglesNode(vertices, color: UIColor.green.withAlphaComponent(1.0))
    }
    
    func getRectPoints(_ p0: CPoint, _ p1: CPoint, _ width: Double, _ zOffset: Double = 0.05) -> [SCNVector3] {
        let theta0 = p0.theta
        let theta1 = p1.theta
        let a = CGeoPoint(x: p0.x - width*sin(theta0), y: p0.y + width*cos(theta0), z: p0.z + zOffset)
        let b = CGeoPoint(x: p0.x + width*sin(theta0), y: p0.y - width*cos(theta0), z: p0.z + zOffset)
        let c = CGeoPoint(x: p1.x - width*sin(theta1), y: p1.y + width*cos(theta1), z: p1.z + zOffset)
        let d = CGeoPoint(x: p1.x + width*sin(theta1), y: p1.y - width*cos(theta1), z: p1.z + zOffset)
        return drawFace([a, b, c, d])
    }
    
    func getNextPoint(_ lane: CLane, _ index: Int, _ t: Double, _ len: Double) -> (CPoint, Int, Double) {
        var iindex = index
        var tt = t
        var p = lane.points[Int(lane.points_size) - 1]
        if iindex >= lane.points_size-1 {return (p, iindex, 0.0)}
        while iindex < lane.points_size - 1 {
            if lane.points[iindex + 1].s > len {break;}
            iindex += 1
        }
        if iindex == lane.points_size - 1 {return (p, iindex, 0.0)}
        tt = 1 - (lane.points[iindex+1].s - len) / (lane.points[iindex+1].s - lane.points[iindex].s)
        p.x = (1-tt)*lane.points[iindex].x + tt*lane.points[iindex+1].x
        p.y = (1-tt)*lane.points[iindex].y + tt*lane.points[iindex+1].y
        p.z = (1-tt)*lane.points[iindex].z + tt*lane.points[iindex+1].z
        p.s = (1-tt)*lane.points[iindex].s + tt*lane.points[iindex+1].s
        var theta1 = lane.points[iindex].theta
        var theta2 = lane.points[iindex+1].theta
        while theta1 - theta2 > Double.pi {theta1 -= 2*Double.pi}
        while theta2 - theta1 > Double.pi {theta2 -= 2*Double.pi}
        p.theta = (1-tt) * theta1 + tt*theta2
        return (p, iindex, tt)
    }
    
    func drawRoadObjects(objects : CRoadObjects) -> [SCNNode] {
        var result = [SCNNode]()
        for i in 0 ..< objects.objs_size {
            if (String(cString: objects.objs[Int(i)].type) == "stopLine") {
                result += [drawStopLine(objects.objs[Int(i)])]
            }
            else if (String(cString: objects.objs[Int(i)].type) == "crosswalk") {
                //drawCrosswalk(objects.objs[Int(i)])
            }
            else if (String(cString: objects.objs[Int(i)].type) == "parkingSpace") {
                result += drawParkingSpace(objects.objs[Int(i)])
            }
            else if (String(cString: objects.objs[Int(i)].type) == "normalObject") {
                result += [drawNormalObject(objects.objs[Int(i)])]
            }
            else if (String(cString: objects.objs[Int(i)].type) == "freeSpace") {
                //drawFreeSpace(objects.objs[Int(i)])
            }
            else if (String(cString: objects.objs[Int(i)].type) == "station") {
                result += drawStation(objects.objs[Int(i)])
            }
        }
        return result
    }
    
    func drawStopLine(_ msg : CRoadObject) -> SCNNode {
        var p1 = msg.polygon[0]
        var p2 = msg.polygon[1]
        p1.z = msg.position.z + 0.05
        p2.z = msg.position.z + 0.05
        let vertices = drawLine(p1, p2, 0.2);
        return getTranglesNode(vertices, color: UIColor(red: 1.0, green: 1.0, blue: 1.0, alpha: 1.0))
    }
    
    func drawParkingSpace(_ msg : CRoadObject) -> [SCNNode] {
        var vertices1 = [SCNVector3]()
        var vertices2 = [SCNVector3]()
        let points : [CGeoPoint] = [msg.polygon[0], msg.polygon[1], msg.polygon[2], msg.polygon[3]]
        vertices1 += drawFace(points)
        var p1, p2 : CGeoPoint
        for i in 0 ..< msg.polygon_size {
            p1 = msg.polygon[Int(i)];
            p2 = msg.polygon[Int(i+1) % Int(msg.polygon_size)]
            p1.z += 0.05
            p2.z += 0.05
            vertices2 += drawLine(p1, p2, 0.15)
        }
        var result = [SCNNode]()
        result += [getTranglesNode(vertices1, color: UIColor(red: 0.05, green: 0.60, blue: 0.05, alpha: 1.0))]
        result += [getTranglesNode(vertices2, color: UIColor(red: 1.0, green: 1.0, blue: 0.0, alpha: 1.0))]
        return result
    }
    
    func drawNormalObject(_ msg: CRoadObject) -> SCNNode {
        let height = msg.height == 0.0 ? 2.0 : msg.height
        var vertices = [SCNVector3]()
        var points = [CGeoPoint]()
        var points_u = [CGeoPoint]()
        let n : Int = Int(msg.polygon_size)
        for i in 0 ..< n {
            points += [msg.polygon[i]]
            points[i].z += 0.02
            points_u += [msg.polygon[i]]
            points_u[i].z += height
        }
        vertices += drawFace(points)
        vertices += drawFace(points_u)
        points += [points[0]]
        points_u += [points_u[0]]
        for i in 0 ... n {
            vertices += drawFace([points[i], points[i + 1], points_u[i+1], points_u[i]])
        }
        return getTranglesNode(vertices, color: UIColor(red: 0.20, green: 0.20, blue: 0.20, alpha: 1.0))
    }
    
    func drawStation(_ msg : CRoadObject) -> [SCNNode] {
        var vertices1 = [SCNVector3]()
        var vertices2 = [SCNVector3]()
        let points : [CGeoPoint] = [msg.polygon[0], msg.polygon[1], msg.polygon[2], msg.polygon[3]]
        vertices1 += drawFace(points)
        var p1, p2 : CGeoPoint
        for i in 0 ..< msg.polygon_size {
            p1 = msg.polygon[Int(i)];
            p2 = msg.polygon[Int(i+1) % Int(msg.polygon_size)]
            p1.z += 0.05
            p2.z += 0.05
            vertices2 += drawLine(p1, p2, 0.15)
        }
        var result = [SCNNode]()
        result += [getTranglesNode(vertices1, color: UIColor(red: 0.05, green: 0.60, blue: 0.05, alpha: 1.0))]
        result += [getTranglesNode(vertices2, color: UIColor(red: 1.0, green: 1.0, blue: 1.0, alpha: 1.0))]
        return result
    }
    
    func getTranglesNode(_ vertices : [SCNVector3], color : UIColor) -> SCNNode {
        let vCount = vertices.count
        let num: Int = vCount / 3
        
        let indices : [Int32] = [Int32](Int32(0) ..< Int32(vCount))
        let verticesSource = SCNGeometrySource(vertices : vertices)
        let normalsSource = SCNGeometrySource(normals : Array<SCNVector3>(repeating : SCNVector3Make(0, 0, 1), count : vCount) )
        let texSource = SCNGeometrySource(textureCoordinates : [CGPoint(x:0, y:0), CGPoint(x:0, y:1), CGPoint(x:1, y:0)])
        
        let indexData = Data(bytes : indices, count : vCount * MemoryLayout<Int32>.size)
        let element = SCNGeometryElement(data: indexData, primitiveType: .triangles, primitiveCount: num, bytesPerIndex: MemoryLayout<Int32>.size)
        let geometry = SCNGeometry(sources: [verticesSource, normalsSource, texSource], elements: [element])
        
        let material = SCNMaterial()
        material.diffuse.contents = color
        material.isDoubleSided = true
        geometry.materials = [material]
        let res = SCNNode(geometry : geometry)
        return res
    }
}
