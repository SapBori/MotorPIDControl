
// RobotExp_4Dlg.cpp : ���� ����
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "RobotExp_4Dlg.h"
#include "afxdialogex.h"
#include "DataType.h"
#include <math.h>
#include "SystemMemory.h"
#include "SharedMemory.h"
#include "stdlib.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.
int mode = 0;
extern int flag = 99;
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

// �����Դϴ�.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRobotExp_4Dlg ��ȭ ����



CRobotExp_4Dlg::CRobotExp_4Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ROBOTEXP_4_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotExp_4Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_PORT, m_ComboPort);
	DDX_Control(pDX, IDC_COMBO_BAUD, m_ComboBaud);
	DDX_Control(pDX, IDC_CHECK_OPEN, m_CheckOpen);
	DDX_Control(pDX, IDC_EDIT_SEND, m_EditSend);
	DDX_Control(pDX, IDC_EDIT_RECV, m_EditRecv);
	DDX_Control(pDX, IDC_EDIT_TAR_POS_1, m_editTarPos1);
	DDX_Control(pDX, IDC_EDIT_TAR_POS_2, m_editTarPos2);
	DDX_Control(pDX, IDC_EDIT_TAR_Vel, m_editTarVel);
	DDX_Control(pDX, IDC_EDIT_TAR_Torq, m_editTarTorq);
	DDX_Control(pDX, IDC_EDIT_TAR_Vel, m_editTarVel);
	DDX_Control(pDX, IDC_EDIT_TAR_Torq, m_editTarTorq);
	DDX_Control(pDX, IDC_EDIT_CUR_Vel, m_editCurVel);
	DDX_Control(pDX, IDC_EDIT_CUR_Torq, m_editCurTorq);
	DDX_Control(pDX, IDC_EDIT_CUR_POS_1, m_editCurPos1);
	DDX_Control(pDX, IDC_EDIT_CUR_POS2, m_editCurPos2);
	DDX_Control(pDX, IDC_EDIT_TAR_X, m_editTarX);
	DDX_Control(pDX, IDC_EDIT_TAR_Y, m_editTarY);
	DDX_Control(pDX, IDC_EDIT_TAR_Z, m_editTarZ);
	DDX_Control(pDX, IDC_EDIT_CUR_POS_X, m_editCurX);
	DDX_Control(pDX, IDC_EDIT_CUR_POS_Y, m_editCurY);
	DDX_Control(pDX, IDC_EDIT_CUR_POS_Z, m_editCurZ);

	DDX_Control(pDX, IDC_CHECK_Mode, m_editMobile);
}

BEGIN_MESSAGE_MAP(CRobotExp_4Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_CBN_DROPDOWN(IDC_COMBO_PORT, &CRobotExp_4Dlg::OnCbnDropdownComboPort)
	ON_BN_CLICKED(IDC_CHECK_OPEN, &CRobotExp_4Dlg::OnBnClickedCheckOpen)
	ON_BN_CLICKED(IDC_BTN_SEND, &CRobotExp_4Dlg::OnBnClickedBtnSend)
	ON_BN_CLICKED(IDC_BTN_CLEAR, &CRobotExp_4Dlg::OnBnClickedBtnClear)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_INIT, &CRobotExp_4Dlg::OnBnClickedButtonInit)
	ON_BN_CLICKED(IDC_BUTTON_FORWARD, &CRobotExp_4Dlg::OnBnClickedButtonForward)
	ON_BN_CLICKED(IDC_BUTTON_INVERSE, &CRobotExp_4Dlg::OnBnClickedButtonInverse)
	ON_BN_CLICKED(IDC_BUTTON_GRAPH, &CRobotExp_4Dlg::OnBnClickedButtonGraph)
	ON_BN_CLICKED(IDC_BUTTON_Set, &CRobotExp_4Dlg::OnBnClickedButtonSet)
	ON_BN_CLICKED(IDC_CHECK_Mode, &CRobotExp_4Dlg::OnBnClickedCheckMode)
END_MESSAGE_MAP()


// CRobotExp_4Dlg �޽��� ó����

BOOL CRobotExp_4Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	SetTimer(1, 100, NULL);
	m_editTarPos1.SetWindowTextA("0");
	m_editTarPos2.SetWindowTextA("0");
	m_editTarVel.SetWindowTextA("0");
	m_editTarTorq.SetWindowTextA("0");
	m_editTarX.SetWindowTextA("0.0");
	m_editTarY.SetWindowTextA("0.0");
	m_editTarZ.SetWindowTextA("0.0");
	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.
	m_pGraphDlg = new CGraphDlg;
	m_pGraphDlg->Create(IDD_GRAPH_DIALOG);

	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// �� ��ȭ ������ �������� �����մϴ�.  ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);			// ū �������� �����մϴ�.
	SetIcon(m_hIcon, FALSE);		// ���� �������� �����մϴ�.

	// TODO: ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.
	SetTimer(1001, 33, NULL);
	_commWorker.SetPeriod(0.01);
	_commWorker.SetWork(CreateWork<CCommWork>("Comm1Work"));
	m_editTarX.SetWindowText("0");
	m_editTarY.SetWindowText("0");
	m_editTarZ.SetWindowText("0");

	m_editTarPos1.SetWindowText("0");
	m_editTarPos2.SetWindowText("0");

	m_editTarVel.SetWindowText("10");
	m_editTarTorq.SetWindowText("0.1");




	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
}
void CRobotExp_4Dlg::OnDestroy() {
	CDialogEx::OnDestroy();

	delete m_pGraphDlg;
}
void CRobotExp_4Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}
// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�.  ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CRobotExp_4Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
HCURSOR CRobotExp_4Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}
/*void CRobotExp_4Dlg::OnTimer(double* pdPos) {
	CString strEndPos;
	double a_endPos[3] = { 0, };
	double q1, q2, q3;

	q1 = _pDIg->_ode->GetJointAngle("Joint1", 0);
	q2 = _pDlg->_ode->GetJointAngel("Joint2", 0);
	q3 = 0.0;

	a_endPos[0] = (4967757600021511.0 * cos(q1)) / 2028;
	a_endPos[1] = (2.0 * sin(q1)) / 5.0 + 43.0 * cos(q1);
	a_endPos[2] = (43.0 * sin(q1) * sin(q2)) / 100;
	double IDC_EDIT_END_POS_X = pdPos[0];
	double IDC_EDIT_END_POS_Y = pdPos[1];
	double IDC_EDIT_END_POS_Z = pdPos[2];

	strEndPos.Format("%.2f", a_endPos[0]);
	SetDlgItemText(IDC_EDIT_END_POS_X, strEndPos);
	strEndPos.Format("%.2f", a_endPos[1]);
	SetDlgItemText(IDC_EDIT_END_POS_Y, strEndPos);
	strEndPos.Format("%.2f", a_endPos[2]);
	SetDlgItemText(IDC_EDIT_END_POS_Z, strEndPos);
}
*/
void CRobotExp_4Dlg::SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos)
{
	double i1 = 1.0;
	double i2 = 0.5;
	//pdPos[0] = sin(dAngle) + (cos(dAngle2) * sin(dAngle)) / 2;
	//pdPos[1] = -sin(dAngle2) / 2;
	//pdPos[2] = 0.5 + cos(dAngle) + (cos(dAngle) * cos(dAngle2)) / 2;
	pdPos[0] = i1 * cos(dAngle) + (i2) * cos(dAngle2+dAngle);
	pdPos[1] = i1 * sin(dAngle) + (i2) * sin(dAngle2+dAngle);
	pdPos[2] = 0.5;
}


void CRobotExp_4Dlg::SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle)
{
	double x = dX;
	double y = dY;
	double z = dZ;
	double L1 = 1;
	double L2 = 0.5;
	double s2, c2;
	double s1, c1;

	c2 = (double)(pow(x, 2) + pow(y, 2) - (pow(L1, 2) + pow(L2, 2))) / (2 * L1 * L2);

	s2 = abs(sqrt(1 - pow(c2, 2)));
	c1 = ((L1 + L2 * c2) * x + (L2 * s2 * y)) / (pow(L1 + L2 * c2, 2) + pow(L2 * s2, 2));
	s1 = ((L1 + L2 * c2) * y - (L2 * s2 * x)) / (pow(L1 + L2 * c2, 2) + pow(L2 * s2, 2));

	pdAngle[1] = atan2(s2, c2);
	pdAngle[0] = atan2(s1, c1);
}

void CRobotExp_4Dlg::OnCbnDropdownComboPort()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	CDeviceListReader reader;
	std::vector < std::string> list;

	m_ComboPort.ResetContent();

	reader.UpdateDeviceList("SERIALCOMM");
	reader.GetDeviceList(list);


	for (int i = 0; i < list.size(); i++) {
		m_ComboPort.AddString(list[i].c_str());
	}
}


void CRobotExp_4Dlg::OnBnClickedCheckOpen()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if (m_CheckOpen.GetCheck()) {
		CString port, baud;
		m_ComboPort.GetLBText(m_ComboPort.GetCurSel(), port);
		m_ComboBaud.GetLBText(m_ComboBaud.GetCurSel(), baud);
		int nTmp = atoi(baud.GetBuffer());
		if (((CCommWork*)_commWorker.GetWork())->OpenPort(port.GetBuffer(), nTmp))
		{
			_commWorker.StartWork();
			m_CheckOpen.SetWindowText("Close");
		}
		else {
			AfxMessageBox("Can't Open Port");
			m_CheckOpen.SetCheck(false);
		}

	}
	else {
		_commWorker.StopWork();
		((CCommWork*)_commWorker.GetWork())->ClosePort();
		m_CheckOpen.SetWindowText("Open");
	}
}


void CRobotExp_4Dlg::OnBnClickedBtnSend()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	/*if (m_comm.isOpen()) {
		CString str;
		m_EditSend.GetWindowTextA(str);

		int size = m_comm.Write(str.GetBuffer(), str.GetLength());
		m_EditSend.SetWindowTextA("");
	}*/
}


void CRobotExp_4Dlg::OnBnClickedBtnClear()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_EditRecv.SetWindowTextA("");
}


void CRobotExp_4Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	ControlData_t motor_data;
	DataType_t ode_data;
	GET_SYSTEM_MEMORY("JointData", ode_data);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", motor_data);
	CString str;
	char number[20];
	str.Format("%.4f", ode_data.Q_cur[1] * RAD2DEG);
	m_editCurPos2.SetWindowTextA(str);
	str.Format("%.4f", (motor_data.position * RAD2DEG));
	m_editCurPos1.SetWindowTextA(str);

	str.Format("%.4f", (motor_data.velocity * RAD2DEG));
	m_editCurVel.SetWindowTextA(str);

	str.Format("%.4f", (motor_data.current * 0.0683));
	m_editCurTorq.SetWindowTextA(str);

	double Pcur[3] = { 0, };
	SolveForwardKinematics(motor_data.position, ode_data.Q_cur[1], Pcur);
	str.Format("%.4f", Pcur[0]);
	m_editCurX.SetWindowText(str);
	str.Format("%.4f", Pcur[1]);
	m_editCurY.SetWindowText(str);
	str.Format("%.4f", Pcur[2]);
	m_editCurZ.SetWindowText(str);
	CDialogEx::OnTimer(nIDEvent);
}


void CRobotExp_4Dlg::OnBnClickedButtonInit()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_editTarX.SetWindowTextA("0");
	m_editTarY.SetWindowTextA("0");
	m_editTarZ.SetWindowTextA("0");

	m_editTarPos1.SetWindowTextA("0");
	m_editTarPos2.SetWindowTextA("0");

	m_editTarVel.SetWindowTextA("10");
	m_editTarTorq.SetWindowTextA("0.1");
	// ODE�� Inverse, Forward�� �°� �����̰� ����

	ControlData_t motor_data_tar;
	DataType_t ode_data;
	GET_SYSTEM_MEMORY("JointData", ode_data);

	ode_data.Q_tar[0] = ode_data.Q_tar[1] = 0.;
	SET_SYSTEM_MEMORY("JointData", ode_data);

	motor_data_tar.position = 0.;
	motor_data_tar.velocity = 10 * DEG2RAD;
	motor_data_tar.current = 0.1 / 0.0683;
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data_tar);
}


void CRobotExp_4Dlg::OnBnClickedButtonForward()
{
	flag = 1;
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	char cTmp[10];
	double dTmp[2];
	m_editTarPos1.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_editTarPos2.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);

	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	jointData.Q_tar[0] = dTmp[0] * DEG2RAD;
	jointData.Q_tar[1] = dTmp[1] * DEG2RAD;

	SET_SYSTEM_MEMORY("JointData", jointData);

	double dPos[3] = { 0, 0, 0 };

	SolveForwardKinematics(jointData.Q_tar[0], jointData.Q_tar[1], dPos);

	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", dPos[0]);
	m_editTarX.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dPos[1]);
	m_editTarY.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dPos[2]);
	m_editTarZ.SetWindowTextA(pszTmp);
}



void CRobotExp_4Dlg::OnBnClickedButtonInverse()
{
	flag = 1;
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	char cTmp[10];
	double dTmp[3];
	m_editTarX.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_editTarY.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);
	m_editTarZ.GetWindowTextA(cTmp, 10);
	dTmp[2] = atof(cTmp);
	double dAngle[2] = { 0, 0 };
	SolveInverseKinematics(dTmp[0], dTmp[1], dTmp[2], dAngle);
	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", dAngle[0] * RAD2DEG);
	m_editTarPos1.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", dAngle[1] * RAD2DEG);
	m_editTarPos2.SetWindowTextA(pszTmp);
	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);
	jointData.Q_tar[0] = dAngle[0];
	jointData.Q_tar[1] = dAngle[1];
	SET_SYSTEM_MEMORY("JointData", jointData);
}



void CRobotExp_4Dlg::OnBnClickedButtonGraph()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	BOOL bCheck = m_pGraphDlg->IsWindowVisible();
	if (bCheck) m_pGraphDlg->ShowWindow(SW_HIDE);
	else m_pGraphDlg->ShowWindow(SW_SHOW);
}



void CRobotExp_4Dlg::OnBnClickedButtonSet()
{
	flag = 25;
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	DataType_t ode_data;
	ControlData_t motor_data_tar;
			// ���Ϳ� ���� �����̰� ODE �۵�
	GET_SYSTEM_MEMORY("JointData", ode_data);
	int temp = 0;
	CString str;
	m_editTarPos1.GetWindowText(str);
	ode_data.Q_tar[0] = atof(str.GetBuffer()) * DEG2RAD;
	m_editTarPos2.GetWindowText(str);
	ode_data.Q_tar[1] = atof(str.GetBuffer()) * DEG2RAD;
	m_editTarPos1.GetWindowText(str);
	temp = atof(str.GetBuffer());
	if (mode == 1) {
		// �������
		if (temp >= 360 || temp<0)
			temp = temp % 360;	
	}
	motor_data_tar.position = temp * DEG2RAD;	//Mobile ���(if���� �Ȱ�ġ��) 
	m_editTarVel.GetWindowText(str);
	motor_data_tar.velocity = atof(str.GetBuffer()) * DEG2RAD;;
	m_editTarTorq.GetWindowText(str);
	motor_data_tar.current = atof(str.GetBuffer())/0.0683;
	SET_SYSTEM_MEMORY("JointData", ode_data);
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", motor_data_tar);
}
	



void CRobotExp_4Dlg::OnBnClickedCheckMode()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	
	if (mode == 0) {
		m_editMobile.SetWindowText("Joint");
		mode = 1;
	}
	else
	{		
		m_editMobile.SetWindowText("Mobile");
		mode = 0;
	}
}
