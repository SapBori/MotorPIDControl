// GraphDlg.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "GraphDlg.h"
#include "afxdialogex.h"
#include "RobotExp_4Dlg.h"


// CGraphDlg ��ȭ �����Դϴ�.

IMPLEMENT_DYNAMIC(CGraphDlg, CDialogEx)

CGraphDlg::CGraphDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_GRAPH_DIALOG, pParent)
{

}

CGraphDlg::~CGraphDlg()
{
}

void CGraphDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_NTGRAPH_TORQ, m_ntgTorque);
	DDX_Control(pDX, IDC_NTGRAPH_Vel, m_ntgVel);
	DDX_Control(pDX, IDC_NTGRAPH_POS, m_ntgPos);
}


BEGIN_MESSAGE_MAP(CGraphDlg, CDialogEx)
	ON_WM_TIMER()
END_MESSAGE_MAP()
BOOL CGraphDlg::OnInitDialog() {
	CDialogEx::OnInitDialog();

	InitNTGraph();
	return TRUE;
}

// CGraphDlg �޽��� ó�����Դϴ�.

void CGraphDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� ��
	m_dCnt += 0.1;
	DataType_t jointData;
	ControlData_t motor_data;
	ControlData_t motor_data_tar;
	GET_SYSTEM_MEMORY("JointData", jointData);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", motor_data);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data_tar);
	if (m_dCnt >= 10.0)
	{
		m_ntgPos.SetRange(m_dCnt - 10.0, m_dCnt, -360.0, 360.0);
		m_ntgVel.SetRange(m_dCnt - 10.0, m_dCnt, -150, 150.0);
		m_ntgTorque.SetRange(m_dCnt - 10.0, m_dCnt, -1.2, 1.2);
	}
	m_ntgPos.PlotXY(m_dCnt, motor_data.position*RAD2DEG,
		2);
	m_ntgPos.PlotXY(m_dCnt, motor_data_tar.position*RAD2DEG,
		1);
	m_ntgVel.PlotXY(m_dCnt, motor_data.velocity*RAD2DEG,
		2);
	m_ntgVel.PlotXY(m_dCnt, motor_data_tar.velocity*RAD2DEG,
		1);
	m_ntgTorque.PlotXY(m_dCnt, motor_data.current*0.0683,
		2);
	m_ntgTorque.PlotXY(m_dCnt, motor_data_tar.current * 0.0683,
		1);
	CDialogEx::OnTimer(nIDEvent);
}
void CGraphDlg::InitNTGraph()
{
	m_ntgPos.ClearGraph();
	m_ntgVel.ClearGraph();
	m_ntgTorque.ClearGraph();
	m_ntgPos.SetFrameStyle(0);
	m_ntgVel.SetFrameStyle(0);
	m_ntgTorque.SetFrameStyle(0);
	m_ntgPos.SetPlotAreaColor(WHITE);
	m_ntgVel.SetPlotAreaColor(WHITE);
	m_ntgTorque.SetPlotAreaColor(WHITE);
	m_ntgPos.SetShowGrid(TRUE);
	m_ntgVel.SetShowGrid(TRUE);
	m_ntgTorque.SetShowGrid(TRUE);
	m_ntgPos.SetFormatAxisBottom(_T("%.2f"));
	m_ntgVel.SetFormatAxisBottom(_T("%.2f"));
	m_ntgTorque.SetFormatAxisBottom(_T("%.2f"));

	m_ntgPos.SetCaption(_T("��ġ"));
	m_ntgVel.SetCaption(_T("�ӵ�"));
	m_ntgTorque.SetCaption(_T("��ũ"));
	m_ntgPos.SetXLabel(_T("Time[s]"));
	m_ntgVel.SetXLabel(_T("Time[s]"));
	m_ntgTorque.SetXLabel(_T("Time[s]"));
	m_ntgPos.SetYLabel(_T("Degree[deg]"));
	m_ntgVel.SetYLabel(_T("Velocity[deg/s]"));
	m_ntgTorque.SetYLabel(_T("Torque[Nm]"));
	m_ntgPos.AddElement();
	m_ntgPos.SetElementWidth(3);
	m_ntgPos.SetElementLineColor(RED);// Target
	m_ntgPos.AddElement();
	m_ntgPos.SetElementWidth(3);
	m_ntgPos.SetElementLineColor(BLUE); // Current
	m_ntgPos.SetRange(0.0, 10.0, -180.0, 180.0);
	m_ntgPos.SetYGridNumber(8);
	m_ntgVel.AddElement();
	m_ntgVel.SetElementWidth(4);
	m_ntgVel.SetElementLineColor(RED);// Target
	m_ntgVel.AddElement();
	m_ntgVel.SetElementWidth(3);
	m_ntgVel.SetElementLineColor(BLUE); // Current
	m_ntgVel.SetRange(0.0, 10.0, -150.0, 150.0);
	m_ntgVel.SetYGridNumber(6);
	m_ntgTorque.AddElement();
	m_ntgTorque.SetElementWidth(4);
	m_ntgTorque.SetElementLineColor(RED);// Target
	m_ntgTorque.AddElement();
	m_ntgTorque.SetElementWidth(3);
	m_ntgTorque.SetElementLineColor(BLUE); // Current
	SetTimer(1, 100, NULL);

}