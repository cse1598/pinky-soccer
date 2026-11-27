#!/usr/bin/env python3
"""Compare the controller AST against the actual dirty baseline, ignoring diagnostics only."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import tarfile


class WithoutTrace(ast.NodeTransformer):
    def visit_ImportFrom(self,node):
        return None if node.module=='timing_trace' else node

    def visit_Assign(self,node):
        allowed={'timing_trace','future_goal_id','future_due'}
        if all(isinstance(t,ast.Attribute) and isinstance(t.value,ast.Name) and t.value.id=='self' and t.attr in allowed for t in node.targets):return None
        if all(isinstance(t,ast.Name) and t.id in {'scheduled_due','had_offer','was_tracking'} for t in node.targets):return None
        return self.generic_visit(node)

    def visit_Expr(self,node):
        call=node.value
        if isinstance(call,ast.Call):
            if isinstance(call.func,ast.Name) and call.func.id=='p' and call.args and isinstance(call.args[0],ast.Constant) and call.args[0].value=='timing_trace':return None
            owner=call.func.value if isinstance(call.func,ast.Attribute) else None
            if isinstance(owner,ast.Attribute) and isinstance(owner.value,ast.Name) and owner.value.id=='self' and owner.attr=='timing_trace':return None
        return self.generic_visit(node)

    def visit_If(self,node):
        if ast.unparse(node.test)=="self.get_parameter('timing_trace').value":return None
        return self.generic_visit(node)


def verify(baseline,root):
    manifest=json.loads((baseline/'manifest.json').read_text())
    controller='src/pinky_soccer_mpc/pinky_soccer_mpc/mpc_controller.py'
    changed=[];missing=[]
    for name,info in manifest['files'].items():
        path=root/name
        if not path.exists():missing.append(name)
        elif hashlib.sha256(path.read_bytes()).hexdigest()!=info['sha256']:changed.append(name)
    with tarfile.open(baseline/'source.tar.gz') as archive:
        old=ast.dump(ast.parse(archive.extractfile(controller).read()),include_attributes=False)
    new=ast.dump(WithoutTrace().visit(ast.parse((root/controller).read_text())),include_attributes=False)
    unexpected=[p for p in changed if p!=controller and p!='README.md' and p!='src/pinky_soccer_mpc/README.md']
    return dict(baseline_files=len(manifest['files']),missing=missing,changed=changed,
        unexpected_changes=unexpected,controller_ast_equal_after_removing_only_trace_statements=old==new,
        passed=not missing and not unexpected and old==new,
        limits='Diagnostic runtime overhead and exception behavior are not proven by AST comparison; covered by measured runs and tests.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline',type=Path)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    args=parser.parse_args();result=verify(args.baseline,args.root)
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
