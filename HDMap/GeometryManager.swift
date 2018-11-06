//
//  GeometryManager.swift
//  AutoParking
//
//  Created by Lubing Han on 2018/6/3.
//  Copyright © 2018年 Lubing Han. All rights reserved.
//

import Foundation
import UIKit
import SceneKit

func getSCNPoint(_ point : CPoint) -> SCNVector3 {
    return SCNVector3Make(Float(point.x), Float(point.y), Float(point.z))
}

func getSCNPoint(_ point : CGeoPoint) -> SCNVector3 {
    return SCNVector3Make(Float(point.x), Float(point.y), Float(point.z))
}

func drawLine(_ p1 : CGeoPoint, _ p2 : CGeoPoint, _ width : Double) -> [SCNVector3] {
    let half_width = width / 2.0
    let dx = p2.x - p1.x
    let dy = p2.y - p1.y
    let theta = atan2(dy, dx)
    let v1 = getSCNPoint(CGeoPoint(x: p1.x - half_width * sin(theta), y: p1.y + half_width * cos(theta), z: p1.z))
    let v2 = getSCNPoint(CGeoPoint(x: p1.x + half_width * sin(theta), y: p1.y - half_width * cos(theta), z: p1.z))
    let v3 = getSCNPoint(CGeoPoint(x: p2.x - half_width * sin(theta), y: p2.y + half_width * cos(theta), z: p2.z))
    let v4 = getSCNPoint(CGeoPoint(x: p2.x + half_width * sin(theta), y: p2.y - half_width * cos(theta), z: p2.z))
    return [v1, v2, v3, v3, v2, v4]
}

func drawFace(_ points : [CPoint]) -> [SCNVector3] {
    var res = [SCNVector3]()
    let n = points.count
    if n > 2 {
        for i in 0 ... n-2 {
            res += [getSCNPoint(points[i])]
            res += [getSCNPoint(points[i + 1])]
            res += [getSCNPoint(points[(i + 2) % n])]
        }
    }
    return res
}

func drawFace(_ points : [CGeoPoint]) -> [SCNVector3] {
    var res = [SCNVector3]()
    let n = points.count
    if n > 2 {
        for i in 0 ... n-2 {
            res += [getSCNPoint(points[i])]
            res += [getSCNPoint(points[i + 1])]
            res += [getSCNPoint(points[(i + 2) % n])]
        }
    }
    return res
}

