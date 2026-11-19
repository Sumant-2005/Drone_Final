"""Adapt the supplied R2023a PMSM_VectorControl.slx without resaving its SPS library links."""
import argparse
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET


def prop(node, name, value):
    elem = node.find(f"P[@Name='{name}']")
    if elem is None:
        elem = ET.SubElement(node, "P", Name=name)
    elem.text = str(value)


def block(system, sid):
    elem = system.find(f"Block[@SID='{sid}']")
    if elem is None:
        raise ValueError(f"Expected block SID {sid} in the supplied R2023a project")
    return elem


def source(system, sid, label, variable):
    b = block(system, sid)
    position = b.findtext("P[@Name='Position']")
    b.clear()
    b.attrib.update(BlockType="FromWorkspace", SID=str(sid), Name=label)
    for k, v in dict(Position=position, VariableName=variable, Interpolate="off",
                     OutputAfterFinalValue="Holding final value", SampleTime="emrax.flight_step").items():
        prop(b, k, v)


def connect(system, src, dst, port=1):
    line = ET.SubElement(system, "Line")
    prop(line, "Src", f"{src}#out:1")
    prop(line, "Dst", f"{dst}#in:{port}")


def scope_panel():
    system = ET.Element("System")
    prop(system, "Location", "[80 80 1160 820]")
    sid = 220

    def add(kind, name, x, y, width, height, **params):
        nonlocal sid
        sid += 1
        b = ET.SubElement(system, "Block", BlockType=kind, Name=name, SID=str(sid))
        prop(b, "Position", f"[{x} {y} {x+width} {y+height}]")
        for key, val in params.items():
            prop(b, key, val)
        return sid

    groups = [
        ("Speed", "rpm", [("w_ref", "Command"), ("w_m", "Shaft speed")]),
        ("Torque", "Nm", [("TL", "Propeller load"), ("T_e", "Electromagnetic torque")]),
        ("Current", "A peak", [("Iq_ref", "q current demand"), ("Iq", "q current"), ("Id", "d current")])]
    for row, (label, unit, channels) in enumerate(groups):
        y = 50 + row*150
        sources = []
        for index, (tag, name) in enumerate(channels):
            top = y+index*42
            signal = add("From", name, 35, top, 130, 24, GotoTag=tag)
            if tag == "w_m":
                gain = add("Gain", "rad per s to rpm", 225, top, 96, 24, Gain="30/pi")
                connect(system, signal, gain)
                signal = gain
            sources.append(signal)
        mux = add("Mux", label+" channels", 380, y, 8, 72, Inputs=len(sources))
        for j, signal in enumerate(sources, 1):
            connect(system, signal, mux, j)
        sample = add("ZeroOrderHold", label+" samples", 477, y+24, 110, 30, SampleTime="emrax.flight_step")
        scope = add("Scope", label+" - "+unit, 675, y+24, 125, 35, OpenAtSimulationStart="off")
        connect(system, mux, sample)
        connect(system, sample, scope)
    # Serialized systems group block definitions before their lines.
    system[:] = [e for e in system if e.tag != 'Line'] + [e for e in system if e.tag == 'Line']
    return system, sid


def adapt(source_file, destination):
    if source_file.resolve() == destination.resolve():
        raise ValueError("The final drive needs its own output file")
    changed = {}
    with ZipFile(source_file) as source_zip:
        def part(path):
            tree = ET.fromstring(source_zip.read(path))
            changed[path] = tree
            return tree

        def sys(sid):
            return part(f"simulink/systems/system_{sid}.xml")

        model = part("simulink/blockdiagram.xml").find("Model")
        prop(model, "PreLoadFcn", "")
        prop(model, "InitFcn", "setup_emrax;")
        interface = part("simulink/graphicalInterface.xml")
        for entry in interface.findall("ExternalFileReference"):
            if entry.findtext("P[@Name='SID']") == "187":
                interface.remove(entry)
        prop(interface, "NumExternalFileReferences", len(interface.findall("ExternalFileReference")))
        cfg = part("simulink/configSet0.xml")
        for field, val in {"StopTime":"emrax.stop_time", "SaveTime":"off", "SaveOutput":"off"}.items():
            cfg.find(f".//P[@Name='{field}']").text = val
        root = sys("root")
        source(root, 135, "Propeller load - Nm", "shaft_load")
        block(root, 152).set("Name","Drive results")
        def mask(system, sid, name, value):
            prop(block(system,sid).find("InstanceData"),name,value)
        mask(root,92,"Amplitude","emrax.Vdc")
        for name, value in {"Resistance":"emrax.Rs", "dqInductances":"[emrax.Ld emrax.Lq]",
                            "Flux":"emrax.flux", "PolePairs":"emrax.pole_pairs", "Mechanical":"[emrax.J 0 emrax.pole_pairs 0]"}.items():
            mask(root,132,name,value)
        for sid in [100,101]:
            prop(block(root,sid),"Gain","emrax.pole_pairs")
        mask(sys(102),179,"Freq","emrax.fsw")

        speed=sys(40)
        source(speed,187,"Motor speed demand - rpm","shaft_speed_command")
        prop(block(speed,144),"Time","0")
        for sid,param in [(44,"speed_kp"),(45,"speed_ki")]:
            prop(block(speed,sid),"Gain","emrax."+param)
        prop(block(speed,52),"Denominator","[emrax.tau_filter 1]")
        for sid, high, low in [(47,"UpperSaturationLimit","LowerSaturationLimit"),(48,"UpperLimit","LowerLimit")]:
            prop(block(speed,sid),high,"emrax.current_limit")
            prop(block(speed,sid),low,"-emrax.current_limit")
        prop(block(speed,188),"UpperLimit","emrax.speed_limit")
        prop(block(speed,188),"LowerLimit","-emrax.speed_limit")
        error_line=next(line for line in speed.findall("Line") if line.findtext("P[@Name='Src']")=="50#out:1")
        prop(error_line,"Src","208#out:1")
        gain=ET.Element("Block",BlockType="Gain",Name="Speed error to rad per s",SID="208")
        prop(gain,"Position","[670 297 755 323]")
        prop(gain,"Gain","pi/30")
        speed.insert(list(speed).index(speed.find("Line")), gain)
        connect(speed,50,208)

        current=sys(3)
        for sid,key in [(28,"kp_q"),(29,"ki_i"),(30,"kp_d"),(31,"ki_i")]:
            prop(block(current,sid),"Gain","emrax."+key)
        for sid in (34,35):
            prop(block(current,sid),"UpperSaturationLimit","1")
            prop(block(current,sid),"LowerSaturationLimit","-1")
        for sid in (36,37):
            prop(block(current,sid),"UpperLimit","1")
            prop(block(current,sid),"LowerLimit","-1")
        d,q=sys(7),sys(14)
        prop(block(d,8),"Value","emrax.Lq")
        prop(block(q,16),"Value","emrax.Ld")
        prop(block(q,17),"Value","emrax.flux")
        prop(block(q,20),"GotoTag","w_elec")
        for system,sid in [(d,11),(q,21)]:
            prop(block(system,sid),"Gain","2/emrax.Vdc")
        scripts={
            23: """function [Ialpha,Ibeta] = fcn(iR,iY,iB)
Ialpha = (2/3)*(iR - (iY+iB)/2);
Ibeta = (iY-iB)/sqrt(3);""",
            44: """function [mR,mY,mB] = fcn(Valpha,Vbeta)
% Linear sinusoidal PWM. Limit the rotating voltage vector to Vdc/2.
scale = max(1,hypot(Valpha,Vbeta));
a = Valpha/scale;
b = Vbeta/scale;
mR = a;
mY = -a/2 + sqrt(3)*b/2;
mB = -a/2 - sqrt(3)*b/2;"""}
        for index,script in scripts.items():
            part(f"simulink/stateflow/chart_{index}.xml").find(".//P[@Name='script']").text=script
        panel, last = scope_panel()
        changed["simulink/systems/system_152.xml"]=panel
        prop(root,"SIDHighWatermark",last)
        # Remaining bitmaps, libraries, electrical netlist and Stateflow metadata are inherited.
        with ZipFile(destination,"w") as target:
            for item in source_zip.infolist():
                data = (ET.tostring(changed[item.filename],encoding="utf-8",xml_declaration=True)
                        if item.filename in changed else source_zip.read(item.filename))
                target.writestr(deepcopy(item),data)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("original",type=Path)
    parser.add_argument("final",type=Path)
    args=parser.parse_args()
    adapt(args.original,args.final)
