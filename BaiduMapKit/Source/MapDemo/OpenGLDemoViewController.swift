//
//  OpenGLDemoViewController.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/9.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

let vssource: String = "precision mediump float;\n" + "attribute vec4 aPosition;\n" + "void main() {\n" + "    gl_Position = aPosition;\n" + "}\n"
let fssource: String = "precision mediump float;\n" + "void main() {\n" + "    gl_FragColor = vec4(1.0, 0.0, 0.0, 0.5);\n" + "}\n"

struct GLPoint {
    var x: GLfloat = 0
    var y: GLfloat = 0
}

class OpenGLDemoViewController: UIViewController, BMKMapViewDelegate {
    
    @IBOutlet weak var _mapView: BMKMapView!
    
    var mapDidFinishLoad: Bool = false
    var coordinate = [CLLocationCoordinate2D]()
    var glshaderLoaded: Bool = false
    
    var program: GLuint = 0
    var aLocPos: GLuint = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        _mapView.isOverlookEnabled = false
        _mapView.isRotateEnabled = false
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        _mapView.viewWillAppear()
        _mapView.delegate = self
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        _mapView.viewWillDisappear()
        _mapView.delegate = nil
    }
    
    // MARK: - BMKMapViewDelegate
    
    /**
    *地图初始化完毕时会调用此接口
    */
    func mapViewDidFinishLoading(_ mapView: BMKMapView!) {
        coordinate.append(CLLocationCoordinate2DMake(39.965, 116.604))
        coordinate.append(CLLocationCoordinate2DMake(39.865, 116.604))
        coordinate.append(CLLocationCoordinate2DMake(39.865, 116.704))
        coordinate.append(CLLocationCoordinate2DMake(39.965, 116.704))
        mapDidFinishLoad = true
    }
    
    /**
     *地图渲染每一帧画面过程中，以及每次需要重绘地图时（例如添加覆盖物）都会调用此接口
     *@param mapview 地图View
     *@param status 此时地图的状态
     */
    func mapView(_ mapView: BMKMapView!, onDrawMapFrame status: BMKMapStatus!) {
        /*
        *do openGL render
        */

        if mapDidFinishLoad {
            glRender(status)
        }
    }
    
    // MARK: - glRender
    func glRender(_ status: BMKMapStatus) {

        if glshaderLoaded == false {
            glshaderLoaded = LoadShaders()
        }
        
        let centerPoint = BMKMapPointForCoordinate(status.targetGeoPt)
        let maprect = _mapView.visibleMapRect
        var glPoints = [GLPoint]()

        for i in 0...3 {
            let point = BMKMapPointForCoordinate(coordinate[i])
            var glPoint = GLPoint()
            glPoint.x = GLfloat((point.x - centerPoint.x) * 2 / maprect.size.width)
            glPoint.y = GLfloat((-point.y + centerPoint.y) * 2 / maprect.size.height)
            glPoints.append(glPoint)
        }
        
        glUseProgram(program);
        glEnableVertexAttribArray(aLocPos)
        glVertexAttribPointer(aLocPos, 2, GLenum(GL_FLOAT), 0, 0, glPoints)
        glDrawArrays(GLenum(GL_TRIANGLE_FAN), 0, 4)
        glDisableVertexAttribArray(aLocPos)
    }
    
    func compileShader(shadersource: String, shader: GLuint) -> Bool {
        var cStringSource = (shadersource as NSString).utf8String
        glShaderSource(shader, 1, &cStringSource, nil)
        
        glCompileShader(shader)
        
        var compiled: GLint = 0
        glGetShaderiv(shader, GLenum(GL_COMPILE_STATUS), &compiled)
        
        if compiled == 0 {
            return false
        }
        
        return true
    }
    
    func LoadShaders() -> Bool {
        let vs = glCreateShader(GLenum(GL_VERTEX_SHADER))
        let fs = glCreateShader(GLenum(GL_FRAGMENT_SHADER))
        
        if !(vs != 0 && fs != 0) {
            return false
        }
        
        if compileShader(shadersource: vssource, shader: vs) == false {
            return false
        }
        
        if compileShader(shadersource: fssource, shader: fs) == false {
            return false
        }
        
        program = glCreateProgram();
        if program == 0 {
            return false
        }
        
        glAttachShader(program, vs)
        glAttachShader(program, fs)
        glLinkProgram(program)
        
        var linked: GLint = 0
        glGetProgramiv(program, GLenum(GL_LINK_STATUS), &(linked))
        
        aLocPos = GLuint(glGetAttribLocation(program, "aPosition"))
        
        if linked == 0 {
            return false
        }

        return true
    }
}


