//
//  ProbeView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 17/07/2022.
//

import SwiftUI

struct ProbeView: View {
    @EnvironmentObject var rl: RL
    var c: Color = .green
    var x_title: String = "Proportional Error"
    var y_title: String = "Intergral Error"
    @State var vals: [Float] = []
    @State var r:Double = 0
    
    var data:[Float] = []
    
    init() {
        print("probe swift init")
    }
    
    
    var body: some View {
        VStack{
            VStack{
                Text("Value")
                valueView.padding()
                Text("Policy")
                policyView.padding()
            }
        }
    }
}


extension ProbeView {
    
    private var valueView: some View {
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
            
            let pixels:Int = 441
            let dim:Int = Int(sqrt(Double(pixels)))
            let x_dif = (g.size.width) / Double(pixels/dim)
            let y_dif = (g.size.height) / Double(pixels/dim)
            
            ForEach((0...440), id: \.self) {
                let y = $0 % dim
                let x = ($0 - y)/dim

                let x_start:Double = x_dif*Double(x)
                let x_end:Double = x_start + x_dif

                let y_start:Double = y_dif*Double(y)
                let y_end:Double = y_start + y_dif

                let ratio = 2 * (rl.probeValues[$0]-rl.probeMinV) / (rl.probeMaxV - rl.probeMinV)
                let b_ = (max(0, 255*(1 - ratio)))
                let r_ = (max(0, 255*(ratio - 1)))
                let g_ = (255) - b_ - r_

                Path { p in
                    p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                    p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                    p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                    p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                    p.closeSubpath()
                }
                .stroke(.black, lineWidth: 2)

                Path { p in
                    p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                    p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                    p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                    p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                    p.closeSubpath()
                }
                .fill(Color(red: Double(r_)/255, green: Double(g_)/255, blue: Double(b_)/255))
            }
        }
    }
}


extension ProbeView {
    
    private var policyView: some View {
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
            
            let pixels:Int = 441
            let dim:Int = Int(sqrt(Double(pixels)))
            let x_dif = (g.size.width) / Double(pixels/dim)
            let y_dif = (g.size.height) / Double(pixels/dim)
            
            ForEach((0...440), id: \.self) {
                let y = $0 % dim
                let x = ($0 - y)/dim

                let x_start:Double = x_dif*Double(x)
                let x_end:Double = x_start + x_dif

                let y_start:Double = y_dif*Double(y)
                let y_end:Double = y_start + y_dif


                let ratio = 2 * (rl.probePolicy[$0]-rl.probeMinP) / (rl.probeMaxP - rl.probeMinP)
                let b_ = (max(0, 255*(1 - ratio)))
                let r_ = (max(0, 255*(ratio - 1)))
                let g_ = (255) - b_ - r_

                Path { p in
                    p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                    p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                    p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                    p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                    p.closeSubpath()
                }
                .stroke(.black, lineWidth: 2)

                Path { p in
                    p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                    p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                    p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                    p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                    p.closeSubpath()
                }
                .fill(Color(red: Double(r_)/255, green: Double(g_)/255, blue: Double(b_)/255))
            }
        }
    }
}

extension RangeExpression where Bound: FixedWidthInteger {
    func randomElements(_ n: Int) -> [Bound] {
        precondition(n > 0)
        switch self {
        case let range as Range<Bound>: return (0..<n).map { _ in .random(in: range) }
        case let range as ClosedRange<Bound>: return (0..<n).map { _ in .random(in: range) }
        default: return []
        }
    }
}

extension Range where Bound: FixedWidthInteger {
    var randomElement: Bound { .random(in: self) }
}

extension ClosedRange where Bound: FixedWidthInteger {
    var randomElement: Bound { .random(in: self) }
}

