//
//  GraphView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 03/05/2022.
//

import SwiftUI

struct GraphView: View {
    
    let title: String
    
    let x1: [Int]
    let y1: [Float]
    
    var x2: [Int] = []
    var y2: [Float] = []
    
    var x3: [Int] = []
    var y3: [Float] = []
    
    var x4: [Int] = []
    var y4: [Float] = []
    
    var x5: [Int] = []
    var y5: [Float] = []
    
    var x6: [Int] = []
    var y6: [Float] = []
    
    var l1: String = "Data1"
    var l2: String = "Data2"
    var l3: String = "Data3"
    var l4: String = "Data4"
    var l5: String = "Data5"
    var l6: String = "Data6"
    
    var c1: Color = .green
    var c2: Color = .blue
    var c3: Color = .red
    var c4: Color = .yellow
    var c5: Color = .cyan
    var c6: Color = .purple
    
    var x_title: String = "Training Steps"
    var y_title: String = "Undiscounted return"
    
    var minManual: Float = 0
    var maxManual: Float = 0

    
    private var key: [String] {
        
        var k1 = ""
        var k2 = ""
        var k3 = ""
        var k4 = ""
        var k5 = ""
        var k6 = ""
        
        if x1.count > 0 {k1 = "-" + l1}
        if x2.count > 0 {k2 = "-" + l2}
        if x3.count > 0 {k3 = "-" + l3}
        if x4.count > 0 {k4 = "-" + l4}
        if x5.count > 0 {k5 = "-" + l5}
        if x6.count > 0 {k6 = "-" + l6}
        
        return [k1, k2, k3, k4, k5, k6]
    }
    
    private var maxVal: Float {
        let yMax1 = y1.max() ?? 0
        let yMax2 = y2.max() ?? 0
        let yMax3 = y3.max() ?? 0
        let yMax4 = y4.max() ?? 0
        let yMax5 = y5.max() ?? 0
        let yMax6 = y6.max() ?? 0
        
        var yMaxs = [yMax1];
        if (y2.count > 0){yMaxs.append(yMax2)}
        if (y3.count > 0){yMaxs.append(yMax3)}
        if (y4.count > 0){yMaxs.append(yMax4)}
        if (y5.count > 0){yMaxs.append(yMax5)}
        if (y6.count > 0){yMaxs.append(yMax6)}
        if (maxManual != 0){yMaxs.append(maxManual)}
        return yMaxs.max() ?? 0
    }
    
    private var minVal: Float {
        let yMin1 = y1.min() ?? 0
        let yMin2 = y2.min() ?? 0
        let yMin3 = y3.min() ?? 0
        let yMin4 = y4.min() ?? 0
        let yMin5 = y5.min() ?? 0
        let yMin6 = y6.min() ?? 0
//        let yMins = [yMin1, yMin2, yMin3, yMin4, yMin5, yMin6 ]
        
        var yMins = [yMin1];
        if (y2.count > 0){yMins.append(yMin2)}
        if (y3.count > 0){yMins.append(yMin3)}
        if (y4.count > 0){yMins.append(yMin4)}
        if (y5.count > 0){yMins.append(yMin5)}
        if (y6.count > 0){yMins.append(yMin6)}
        if (minManual != 0){yMins.append(minManual)}
        return yMins.min() ?? 0
    }
    
    private var maxValX: Int {
        let xMax1 = x1.max() ?? 0
        let xMax2 = x2.max() ?? 0
        let xMax3 = x3.max() ?? 0
        let xMax4 = x4.max() ?? 0
        let xMax5 = x5.max() ?? 0
        let xMax6 = x6.max() ?? 0
        let xMaxs = [xMax1, xMax2, xMax3, xMax4, xMax5, xMax6]
        return xMaxs.max() ?? 0
    }
    
    
    private var minValX: Int {
        let xMin1 = x1.min() ?? 0
        let xMin2 = x2.min() ?? 0
        let xMin3 = x3.min() ?? 0
        let xMin4 = x4.min() ?? 0
        let xMin5 = x5.min() ?? 0
        let xMin6 = x6.min() ?? 0
        
        var xMins = [xMin1];
        if (x2.count > 0){xMins.append(xMin2)}
        if (x3.count > 0){xMins.append(xMin3)}
        if (x4.count > 0){xMins.append(xMin4)}
        if (x5.count > 0){xMins.append(xMin5)}
        if (x6.count > 0){xMins.append(xMin6)}
        
        return xMins.min() ?? 0
    }
    
    
    var body: some View {
        VStack{
            VStack{
                Text(title)
                    .foregroundColor(.gray)
                    .padding(.all, 2)
                    .font(.headline)
            }
            
            HStack{
                Text(y_title)
                    .rotationEffect(.degrees(-90))
                    .fixedSize()
                    .frame(width: 20, height: 180)
                    .foregroundColor(.gray)
                VStack{
                    Text(String(format: "%.1f", maxVal))
                    Spacer()
                    Spacer()
                    Text(String(format: "%.1f", (maxVal+minVal)/2))
                    Spacer()
                    Spacer()
                    Text(String(format: "%.1f", minVal))
                }.font(.caption)
                    .foregroundColor(.gray)
                ZStack {
                    gridView
                    if (y1.count > 1) {lineView1}
                    if (y2.count > 1) {lineView2}
                    if (y3.count > 1) {lineView3}
                    if (y4.count > 1) {lineView4}
                    if (y5.count > 1) {lineView5}
                    if (y6.count > 1) {lineView6}
                    legendView1
//                    Text(key[1])
//                    Text(key[2])
//                    Text(key[3])
//                    Text(key[4])
//                    Text(key[5])
                    
  
//                    legendView1
                    
                }
            }
            HStack{
                Spacer()
                Text(String(maxValX))
                    .font(.caption)
                    .foregroundColor(.gray)
            }
            Text(x_title)
                .foregroundColor(.gray)
        }
    }
}


extension GraphView {
    
    private var gridView: some View {
        GeometryReader { g in
            Path { p in

                let TL = CGPoint(x:0, y:0)
                let TR = CGPoint(x:g.size.width, y:0)
                let BR = CGPoint(x:g.size.width, y:g.size.height)
                let BL = CGPoint(x:0, y:g.size.height)
                    
                p.move(to: TL)
                p.addLine(to: TR)
                p.addLine(to: BR)
                p.addLine(to: BL)
                p.closeSubpath()
            }
            .stroke(.gray, style: StrokeStyle(lineWidth: 1, lineCap: .round, lineJoin: .round))
            
            Path { p in
                p.move(to: CGPoint(x: 0, y: g.size.height/2))
                p.addLine(to: CGPoint(x: g.size.width, y: g.size.height/2))
                
                p.move(to: CGPoint(x: 0, y: g.size.height/4))
                p.addLine(to: CGPoint(x: g.size.width, y: g.size.height/4))
                
                p.move(to: CGPoint(x: 0, y: g.size.height*3/4))
                p.addLine(to: CGPoint(x: g.size.width, y: g.size.height*3/4))
            }
            .stroke(.gray, style: StrokeStyle(lineWidth: 0.5, lineCap: .round, lineJoin: .round))
        }
    }
}

extension GraphView {
    private var legendView1: some View {
        HStack{
            VStack{
                Text(key[0]).foregroundColor(c1)
                Text(key[1]).foregroundColor(c2)
                Text(key[2]).foregroundColor(c3)
                Text(key[3]).foregroundColor(c4)
                Text(key[4]).foregroundColor(c5)
                Text(key[5]).foregroundColor(c6)
            }.padding()
            Spacer()
        }
    }
}


// 1 ===========================================================================================
extension GraphView {
    private var lineView1: some View {
        GeometryReader { g in
            Path { p in
                var maxSteps = maxValX-minValX
                if maxSteps==0 {maxSteps = 1}
                let ratio = g.size.width/CGFloat(maxSteps)
                
                for i in y1.indices {
                    
                    let xPos = Double(ratio)*Double(x1[i]-minValX)
                    var yPos:Double
                    let yAxis = maxVal - minVal
                    if (yAxis==0){
                        yPos = g.size.height/2
                    } else {
                        yPos = (1-CGFloat((y1[i] - minVal) / yAxis)) * g.size.height
                    }
                    if i == 0 {p.move(to: CGPoint(x: xPos, y:yPos))}
                    p.addLine(to: CGPoint(x:xPos, y:yPos)) //remove zero start
                }
            }
            .stroke(c1, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))
        }
    }
}

// 2 ===========================================================================================
extension GraphView {
    private var lineView2: some View {
        GeometryReader { g in
            Path { p in
                var maxSteps = maxValX-minValX
                if maxSteps==0 {maxSteps = 1}
                let ratio = g.size.width/CGFloat(maxSteps)
                
                for i in y2.indices {
                    
                    let xPos = Double(ratio)*Double(x2[i]-minValX)
                    var yPos:Double
                    let yAxis = maxVal - minVal
                    if (yAxis==0){
                        yPos = g.size.height/2
                    } else {
                        yPos = (1-CGFloat((y2[i] - minVal) / yAxis)) * g.size.height
                    }
                    if i == 0 {p.move(to: CGPoint(x: xPos, y:yPos))}
                    p.addLine(to: CGPoint(x:xPos, y:yPos)) //remove zero start
                }
            }
            .stroke(c2, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))
        }
    }
}


// 3 ===========================================================================================
extension GraphView {
    private var lineView3: some View {
        GeometryReader { g in
            Path { p in
                var maxSteps = maxValX-minValX
                if maxSteps==0 {maxSteps = 1}
                let ratio = g.size.width/CGFloat(maxSteps)

                for i in y3.indices {

                    let xPos = Double(ratio)*Double(x3[i]-minValX)
                    var yPos:Double
                    let yAxis = maxVal - minVal
                    if (yAxis==0){
                        yPos = g.size.height/2
                    } else {
                        yPos = (1-CGFloat((y3[i] - minVal) / yAxis)) * g.size.height
                    }
                    if i == 0 {p.move(to: CGPoint(x: xPos, y:yPos))}
                    p.addLine(to: CGPoint(x:xPos, y:yPos)) //remove zero start
                }
            }
            .stroke(c3, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))
        }
    }
}


// 4 ===========================================================================================
extension GraphView {
    private var lineView4: some View {
        GeometryReader { g in
            Path { p in
                var maxSteps = maxValX-minValX
                if maxSteps==0 {maxSteps = 1}
                let ratio = g.size.width/CGFloat(maxSteps)

                for i in y4.indices {

                    let xPos = Double(ratio)*Double(x4[i]-minValX)
                    var yPos:Double
                    let yAxis = maxVal - minVal
                    if (yAxis==0){
                        yPos = g.size.height/2
                    } else {
                        yPos = (1-CGFloat((y4[i] - minVal) / yAxis)) * g.size.height
                    }
                    if i == 0 {p.move(to: CGPoint(x: xPos, y:yPos))}
                    p.addLine(to: CGPoint(x:xPos, y:yPos)) //remove zero start
                }
            }
            .stroke(c4, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))
        }
    }
}


// 5 ===========================================================================================
extension GraphView {
    private var lineView5: some View {
        GeometryReader { g in
            Path { p in
                var maxSteps = maxValX-minValX
                if maxSteps==0 {maxSteps = 1}
                let ratio = g.size.width/CGFloat(maxSteps)

                for i in y5.indices {

                    let xPos = Double(ratio)*Double(x5[i]-minValX)
                    var yPos:Double
                    let yAxis = maxVal - minVal
                    if (yAxis==0){
                        yPos = g.size.height/2
                    } else {
                        yPos = (1-CGFloat((y5[i] - minVal) / yAxis)) * g.size.height
                    }
                    if i == 0 {p.move(to: CGPoint(x: xPos, y:yPos))}
                    p.addLine(to: CGPoint(x:xPos, y:yPos)) //remove zero start
                }
            }
            .stroke(c5, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))
        }
    }
}


// 6 ===========================================================================================
extension GraphView {
    private var lineView6: some View {
        GeometryReader { g in
            Path { p in
                var maxSteps = maxValX-minValX
                if maxSteps==0 {maxSteps = 1}
                let ratio = g.size.width/CGFloat(maxSteps)

                for i in y6.indices {

                    let xPos = Double(ratio)*Double(x6[i]-minValX)
                    var yPos:Double
                    let yAxis = maxVal - minVal
                    if (yAxis==0){
                        yPos = g.size.height/2
                    } else {
                        yPos = (1-CGFloat((y6[i] - minVal) / yAxis)) * g.size.height
                    }
                    if i == 0 {p.move(to: CGPoint(x: xPos, y:yPos))}
                    p.addLine(to: CGPoint(x:xPos, y:yPos)) //remove zero start
                }
            }
            .stroke(c6, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))
        }
    }
}
