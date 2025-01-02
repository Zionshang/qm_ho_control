#include "HoQp.h"

HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : _task(std::move(task)), _higherProblem(std::move(higherProblem))
{
    initVars();
    formulateProblem();
    solveProblem();
    // prepare for next problem
    buildZMatrix();
    stackSlackSolutions();
}

void HoQp::initVars()
{
    // Task variables
    _dimSlackVars = _task._f.rows();
    _hasEqConstraints = _task._b.rows() > 0;
    _hasIneqConstraints = _dimSlackVars > 0;

    // Pre-Task variables
    if (_higherProblem == nullptr) // The task with the highest priority should be solved first
    {

        _dimDecisionVars = _task._A.cols();
        _stackedTasksPrev = Task(_dimDecisionVars);
        _stackedZPrev = MatX::Identity(_dimDecisionVars, _dimDecisionVars);
        _stackedSlackSolPrev = VecX::Zero(0);
        _xPrev = VecX::Zero(_dimDecisionVars);
        _dimPrevSlackVars = 0;
    }
    else
    {
        _stackedZPrev = _higherProblem->getStackedZMatrix();
        _stackedTasksPrev = _higherProblem->getStackedTasks();
        _stackedSlackSolPrev = _higherProblem->getStackedSlackSol();
        _xPrev = _higherProblem->getSolutions();
        _dimPrevSlackVars = _higherProblem->getStackedTasks()._D.rows();
        _dimDecisionVars = _stackedZPrev.cols();
    }

    _stackedTasks = _task + _stackedTasksPrev;

    // Init convenience matrices
    _eyeNvNv = MatX::Identity(_dimSlackVars, _dimSlackVars);
    _zeroNvNx = MatX::Zero(_dimSlackVars, _dimDecisionVars);
}

void HoQp::formulateProblem()
{
    buildHMatrix();
    buildCVector();
    buildDMatrix();
    buildFVector();
}

void HoQp::buildHMatrix()
{
    MatX ZtAtAZ(_dimDecisionVars, _dimDecisionVars); // t means transpose
    if (_hasEqConstraints)
    {
        // Make sure that all eigenvalues of A_t_A are non-negative, which could arise due to numerical issues
        MatX ACurrZPrev = _task._A * _stackedZPrev;
        // This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
        ZtAtAZ = ACurrZPrev.transpose() * ACurrZPrev + 1e-12 * MatX::Identity(_dimDecisionVars, _dimDecisionVars);
    }
    else
        ZtAtAZ.setZero();

    _H = (MatX(_dimDecisionVars + _dimSlackVars, _dimDecisionVars + _dimSlackVars) // clang-format off
              <<   ZtAtAZ, _zeroNvNx.transpose(), 
                _zeroNvNx, _eyeNvNv) // clang-format on
             .finished();
}

void HoQp::buildCVector()
{
    VecX c = VecX::Zero(_dimDecisionVars + _dimSlackVars);
    VecX zeroVec = VecX::Zero(_dimSlackVars);
    VecX temp(_dimDecisionVars);
    if (_hasEqConstraints)
        temp = (_task._A * _stackedZPrev).transpose() * (_task._A * _xPrev - _task._b);
    else
        temp.setZero();

    _c = (VecX(_dimDecisionVars + _dimSlackVars) // clang-format off
             << temp, 
                zeroVec) // clang-format on
             .finished();
}

void HoQp::buildDMatrix()
{
    MatX stackedZero = MatX::Zero(_dimPrevSlackVars, _dimSlackVars);
    MatX DCurrZ;
    if (_hasIneqConstraints)
        DCurrZ = _task._D * _stackedZPrev;
    else
        DCurrZ = MatX::Zero(0, _dimDecisionVars);

    // NOTE: This is upside down compared to the paper, but more consistent with the rest of the algorithm
    _D = (MatX(2 * _dimSlackVars + _dimPrevSlackVars, _dimDecisionVars + _dimSlackVars) // clang-format off
              << _zeroNvNx                           , -_eyeNvNv,
                 _stackedTasksPrev._D * _stackedZPrev, stackedZero,
                 DCurrZ                              , -_eyeNvNv) // clang-format on
             .finished();
}

void HoQp::buildFVector()
{
    VecX zeroVec = VecX::Zero(_dimSlackVars);
    VecX fMinusDXPrev;
    if (_hasIneqConstraints)
        fMinusDXPrev = _task._f - _task._D * _xPrev;
    else
        fMinusDXPrev = VecX::Zero(0);

    _f = (VecX(2 * _dimSlackVars + _dimPrevSlackVars) // clang-format off
              << zeroVec,
                _stackedTasksPrev._f - _stackedTasksPrev._D * _xPrev + _stackedSlackSolPrev,
                fMinusDXPrev)
             .finished(); // clang-format on
}

void HoQp::buildZMatrix()
{
    if (_hasEqConstraints)
        _stackedZ = _stackedZPrev * (_task._A * _stackedZPrev).fullPivLu().kernel();
    else
        _stackedZ = _stackedZPrev;
}

void HoQp::solveProblem()
{
    auto qpProblem = qpOASES::QProblem(_dimDecisionVars + _dimSlackVars, _f.size());
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qpProblem.setOptions(options);
    int nWsr = 20;

    qpProblem.init(_H.data(), _c.data(), _D.data(), nullptr, nullptr, nullptr, _f.data(), nWsr);
    VecX qpSol(_dimDecisionVars + _dimSlackVars);

    qpProblem.getPrimalSolution(qpSol.data());

    _decisionVarsSol = qpSol.head(_dimDecisionVars);
    _slackVarsSol = qpSol.tail(_dimSlackVars);
}

void HoQp::stackSlackSolutions()
{
    if (_higherProblem != nullptr)
        _stackedSlackSol = Task::concatenateVec(_higherProblem->getStackedSlackSol(), _slackVarsSol);
    else
        _stackedSlackSol = _slackVarsSol;
}