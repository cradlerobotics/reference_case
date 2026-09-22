# converting gwen specs to tl 
# avoid -> G!(a) 
# before(l1,l2) -> (!l2) U l1
# visit(l1) -> F (l1) 
import re 

def translate_avoid(op):
    return ("(G ! ({0}))".format(op))

def translate_before(l1,l2):
    return ("((!({0})) U {1})".format(l2,l1))

def translate_visit(op):
    return ("(F {0})".format(op))

def combine(reqs):
    return ' AND '.join(reqs)

def bracket(op):
    return ("({0})".format(op))

def translate_req(req):
    visit_re="visit\((.*)\)"
    avoid_re="avoid\((.*)\)"
    before_re="before\((.*),(.*)\)"
    
    translated=None
    res = [visit_re,avoid_re,before_re]
    for re_pats in res:
        match = re.search(re_pats,req)
        if match:
            if re_pats is visit_re:
                
                translated=translate_visit(match.group(1))
            elif re_pats is avoid_re:
                translated=translate_avoid(match.group(1))
            elif re_pats is before_re:
                translated=translate_before(match.group(1),match.group(2))
            else:
                print("No function to translate pattern {0}:{1}".format(re_pats,req))
    
    return translated
                
        
    
def translate_gwen_list(req_list):
    # stuff
    ts = [] 
    for req in req_list:
        ts.append(translate_req(req))
        

        
    return ts

def gwen_spec_extraction(gwenspec):
    speclist=[]
    search_pattern = "specification\(\".*\"\,\[(.*)\]\)"
    match = re.search(search_pattern,gwenspec)
    if match:
        speclistline = match.group(1)
        speclist = re.split(",\s*(?![^()]*\))",speclistline)
        #speclist = speclistline.split(',')
        
    return speclist

def do_gwen_specs():
    specs1 = ["specification(\"Required\",[visit(pipes)])","specification(\"Preferred\",[avoid(danger_orange)])"]
    specs2 = ["specification(\"Required\",[visit(ipone),visit(t2bottom),visit(tankset),visit(pipes)])", "specification(\"Preferred\",[avoid(danger_red),before(ipone,t2bottom),before(ipone,tankset),before(ipone,pipes),before(t2bottom,pipes),before(tankset,pipes),before(t2bottom,tankset)])"]
    specs3 = ["specification(\"Required\",[visit(ipone),visit(t2bottom),visit(tankset),visit(pipes)])", "specification(\"Preferred\",[avoid(t2left),before(ipone,t2bottom),before(ipone,tankset),before(ipone,pipes),before(t2bottom,pipes),before(tankset,pipes),before(t2bottom,tankset)])"]
    specs = [specs1,specs2,specs3]
    for spec in specs:
        print("==============================================================")
        for part in spec:
            print("Translating {0}".format(part))
            speclist = gwen_spec_extraction(part)
            translated = translate_gwen_list(speclist)
            combined = combine(translated)
            print(combined)
            print("---------------------------------------------------------")
        print("==============================================================")
            
if __name__=="__main__":
    do_gwen_specs()                    