#!/usr/bin/env python3
"""Merge successful floor tests into Excel using the standard library only."""
import argparse
import collections
import datetime
import json
from pathlib import Path
import statistics
import zipfile
from xml.sax.saxutils import escape

NS='http://schemas.openxmlformats.org/spreadsheetml/2006/main'


def excel_col(number):
    label=''
    while number:
        number,rem=divmod(number-1,26);label=chr(65+rem)+label
    return label


def worksheet(rows,widths=None):
    widths=widths or [min(72,max(14,max(len(str(row[i])) if i<len(row) else 0 for row in rows[:50])+2)) for i in range(max(map(len,rows)))]
    parts=[f'<worksheet xmlns="{NS}"><sheetViews><sheetView workbookViewId="0"><pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/></sheetView></sheetViews><cols>']
    for i,width in enumerate(widths,1):parts.append(f'<col min="{i}" max="{i}" width="{width}" customWidth="1"/>')
    parts.append('</cols><sheetData>')
    for r,row in enumerate(rows,1):
        parts.append(f'<row r="{r}">')
        for c,value in enumerate(row,1):
            if value is None:continue
            address=f'{excel_col(c)}{r}';style=1 if r==1 else 2 if isinstance(value,(int,float)) and not isinstance(value,bool) else 0
            if isinstance(value,(int,float)) and not isinstance(value,bool):
                parts.append(f'<c r="{address}" s="{style}"><v>{value}</v></c>')
            else:
                text=str(value)
                parts.append(f'<c r="{address}" s="{style}" t="inlineStr"><is><t xml:space="preserve">{escape(text)}</t></is></c>')
        parts.append('</row>')
    parts.append('</sheetData>')
    if len(rows)>1:parts.append(f'<autoFilter ref="A1:{excel_col(max(map(len,rows)))}{len(rows)}"/>')
    parts.append('</worksheet>')
    return ''.join(parts)


def write_xlsx(path,sheets):
    content=['<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/><Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>']
    for i in range(1,len(sheets)+1):content.append(f'<Override PartName="/xl/worksheets/sheet{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>')
    content.append('</Types>')
    workbook=[f'<workbook xmlns="{NS}" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets>']
    rels=['<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">']
    for i,(name,_) in enumerate(sheets,1):
        workbook.append(f'<sheet name="{escape(name)}" sheetId="{i}" r:id="rId{i}"/>')
        rels.append(f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{i}.xml"/>')
    workbook.append('</sheets></workbook>')
    rels.append(f'<Relationship Id="rId{len(sheets)+1}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>')
    styles=f'''<styleSheet xmlns="{NS}"><numFmts count="1"><numFmt numFmtId="164" formatCode="0.000"/></numFmts><fonts count="2"><font><sz val="11"/><name val="Calibri"/></font><font><b/><sz val="11"/><color rgb="FFFFFFFF"/><name val="Calibri"/></font></fonts><fills count="3"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill><fill><patternFill patternType="solid"><fgColor rgb="FF24476B"/><bgColor indexed="64"/></patternFill></fill></fills><borders count="1"><border/></borders><cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs><cellXfs count="3"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"><alignment vertical="top" wrapText="1"/></xf><xf numFmtId="0" fontId="1" fillId="2" borderId="0" xfId="0"><alignment wrapText="1"/></xf><xf numFmtId="164" fontId="0" fillId="0" borderId="0" xfId="0" applyNumberFormat="1"/></cellXfs><cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles></styleSheet>'''
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml',''.join(content))
        z.writestr('_rels/.rels','<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>')
        z.writestr('xl/workbook.xml',''.join(workbook));z.writestr('xl/_rels/workbook.xml.rels',''.join(rels));z.writestr('xl/styles.xml',styles)
        for i,(_,rows) in enumerate(sheets,1):z.writestr(f'xl/worksheets/sheet{i}.xml',worksheet(rows))


def build_sheets(paths):
    runs=[];selected={};attempts=[]
    for path in paths:
        rows=[json.loads(x) for x in path.read_text().splitlines()]
        config=next((r for r in rows if r['kind']=='configuration'),{})
        runs.append((path,rows,config))
        for result in rows:
            if result['kind']!='goal_result':continue
            attempts.append((path,result))
            if result['success']:
                # Sources are ordered oldest first; take latest successful result.
                selected[result['floor']]=(path,result,[r for r in rows if r.get('floor')==result['floor']])
    missing=sorted(set(range(1,20))-set(selected))
    if missing:raise ValueError(f'缺少成功楼层：{missing}')
    interval_rows=[['楼层','上一按键','下一按键','到位间隔(s)','≤2.8s','<3s','上一到位单调时钟(ns)','下一到位单调时钟(ns)','来源日志']]
    floors=[['楼层','最终结果','按键顺序','间隔数','最短(s)','平均(s)','最长(s)','≤2.8s数','<3s数','完成时间','来源日志']]
    stage_rows=[['楼层','按键','阶段','规划轨迹(s)','实际执行(s)','TCP验收(s)','执行加验收(s)','来源日志']]
    planning_rows=[['楼层','预规划总耗时(s)','请求数','失败请求数','请求等待合计(s)','来源日志']]
    values=[]
    for floor,(path,result,rows) in sorted(selected.items()):
        intervals=[r for r in rows if r['kind']=='interval']
        expected=len(str(floor))
        if len(intervals)!=expected:raise ValueError(f'{floor}楼间隔不完整：{len(intervals)}/{expected}')
        local=[float(r['interval_s']) for r in intervals];values.extend(local)
        for r,value in zip(intervals,local):
            interval_rows.append([floor,r['previous_target'],r['target'],value,'是' if value<=2.8 else '否','是' if value<3 else '否',str(r['previous_command_monotonic_ns']),str(r['command_monotonic_ns']),path.name])
        targets=[f'key_{d}' for d in str(floor)]+['key_ok']
        floors.append([floor,'成功',' → '.join(targets),len(local),min(local),statistics.mean(local),max(local),sum(v<=2.8 for v in local),sum(v<3 for v in local),result['time'],path.name])
        timings=[r for r in rows if r['kind']=='segment_timing']
        for r in timings:
            if 'execution_and_verification_s' in r:
                stage_rows.append([floor,r['target'],r['stage'],r.get('planned_duration_s'),r.get('trajectory_execution_s'),r.get('tcp_verification_s'),r['execution_and_verification_s'],path.name])
        requests=[r for r in timings if r.get('timing_kind')=='planning_request']
        total=next((r['elapsed_s'] for r in timings if r.get('timing_kind')=='planning_total'),None)
        planning_rows.append([floor,total,len(requests),sum(not r['success'] for r in requests),sum(r['elapsed_s'] for r in requests),path.name])
    failure_count=sum(not r['success'] for _,r in attempts)
    overview=[['项目','结果','说明'],['成功楼层',len(selected),'1～19，采用各楼层最新成功尝试'],['有效到位间隔',len(values),'仅同楼层数字之间及末位数字到OK'],['最短间隔(s)',min(values),'实测TCP推进完成并验收通过'],['平均间隔(s)',statistics.mean(values),'不是物理按钮触发时间'],['最长间隔(s)',max(values),'完整数值保留于到位间隔表'],['≤2.8s间隔数',sum(v<=2.8 for v in values),'按未四舍五入的数值判断'],['>2.8s间隔数',sum(v>2.8 for v in values),''],['<3s间隔数',sum(v<3 for v in values),''],['≥3s间隔数',sum(v>=3 for v in values),''],['失败尝试数',failure_count,'保留在全部尝试；不混入成功间隔'],['数据拼接规则','第一段1～5成功，第二段6～19成功','6楼第一次失败，采用第二段复测成功结果'],['跨运行间隔','不计算','不伪造5楼到6楼或其他跨运行时间差'],['计时基准','相邻实测按压到位事件','包含OK；不含Ready、视觉采样及运动前规划时间'],['结果含义','软件任务成功及最终回Ready','不证明每个电梯按钮灯亮/硬件开关触发']]
    all_attempts=[['来源日志','楼层','状态码','success','完成时间','hard_safety_stop','结果消息']]
    for path,r in attempts:all_attempts.append([path.name,r['floor'],r['status'],'成功' if r['success'] else '失败',r['time'],str(r['hard_safety_stop']),r['message']])
    params=[['来源日志','节点/区域','参数','实际记录值']]
    sources=[['来源日志','完整路径','记录楼层','用途']]
    for path,rows,cfg in runs:
        sources.append([path.name,str(path.resolve()),', '.join(map(str,cfg.get('floors',[]))),'最新成功按楼层合并；失败另外保留'])
        for group in ('pbvs','sequence','driver','wrist_limits'):
            for name,value in cfg.get(group,{}).items():params.append([path.name,group,name,json.dumps(value,ensure_ascii=False) if isinstance(value,(list,dict)) else str(value)])
        params.append([path.name,'记录定义','timing_reference',cfg.get('timing_reference','measured_tcp_press_target_verified')])
    return [('总览',overview),('楼层汇总',floors),('到位间隔',interval_rows),('阶段耗时',stage_rows),('规划成本',planning_rows),('全部尝试',all_attempts),('实际参数',params),('数据来源',sources)],dict(successful_floors=sorted(selected),interval_count=len(values),maximum_s=max(values),mean_s=statistics.mean(values),over_2_8=sum(v>2.8 for v in values),at_least_3=sum(v>=3 for v in values),failed_attempts=failure_count)


def main():
    p=argparse.ArgumentParser();p.add_argument('logs',nargs='+',type=Path);p.add_argument('--output',required=True,type=Path);args=p.parse_args()
    sheets,stats=build_sheets(args.logs);args.output.parent.mkdir(parents=True,exist_ok=True);write_xlsx(args.output,sheets)
    print(args.output);print(json.dumps(stats,ensure_ascii=False))


if __name__=='__main__':main()
