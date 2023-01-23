//
//  BarView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 31/05/2022.
//
import SwiftUI

struct BarView: View {
    
    let title: String
    let P: Float
    let I: Float
    let D: Float
    
    let limP: Float
    let limI: Float
    let limD: Float
    
    private var maxVal: Float {
        let yMaxs = [limP, limI, limD, abs(P), abs(I), abs(D)]
        return yMaxs.max() ?? 0
    }
    
    private var c1:Color {
        var c: Color
        if (abs(P)>limP){
            c = .red
        } else {
            c = .green
        }
        return c
    }
    
    private var c2:Color {
        var c: Color
        if (abs(I)>limI){
            c = .red
        } else {
            c = .green
        }
        return c
    }
    
    private var c3:Color {
        var c: Color
        if (abs(D)>limD){
            c = .red
        } else {
            c = .green
        }
        return c
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
                VStack{
                    Text(String(format: "%.1f", maxVal))
                    Spacer()
                    Spacer()
                    Text(String(format: "%.1f", (maxVal-maxVal)/2))
                    Spacer()
                    Spacer()
                    Text(String(format: "%.1f", -maxVal))
                }.font(.caption)
                    .foregroundColor(.gray)
                ZStack {
                    gridView
                    PView
                    IView
                    DView
                }
            }
            HStack{
                Text(String(P)).foregroundColor(c1)
                Spacer()
                Text(String(I)).foregroundColor(c2)
                Spacer()
                Text(String(D)).foregroundColor(c3)
            }
        }
    }
}


extension BarView {
    
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


//// 1 ===========================================================================================
extension BarView {
    private var PView: some View {
    
        
        
        GeometryReader { g in
            Path { p in
                let yPos = (1-CGFloat((P + maxVal) / (2*maxVal))) * g.size.height
                p.move(to: CGPoint(x: 0, y: g.size.height/2))
                p.addLine(to: CGPoint(x: 0, y: yPos))
            }
            .stroke(c1, style: StrokeStyle(lineWidth: 10, lineCap: .round, lineJoin: .round))
        }
    }
}


extension BarView {
    private var IView: some View {

        GeometryReader { g in
            Path { p in
                let yPos = (1-CGFloat((I + maxVal) / (2*maxVal))) * g.size.height
                p.move(to: CGPoint(x: g.size.width/2, y: g.size.height/2))
                p.addLine(to: CGPoint(x: g.size.width/2, y: yPos))
            }
            .stroke(c2, style: StrokeStyle(lineWidth: 10, lineCap: .round, lineJoin: .round))
        }
    }
}


extension BarView {
    private var DView: some View {
        GeometryReader { g in
            Path { p in
                let yPos = (1-CGFloat((D + maxVal) / (2*maxVal))) * g.size.height
                p.move(to: CGPoint(x: g.size.width, y: g.size.height/2))
                p.addLine(to: CGPoint(x: g.size.width, y: yPos))
            }
            .stroke(c3, style: StrokeStyle(lineWidth: 10, lineCap: .round, lineJoin: .round))
        }
    }
}
