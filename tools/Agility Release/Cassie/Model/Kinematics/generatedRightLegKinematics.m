function [T,J] = generatedRightLegKinematics(in1,in2)
%GENERATEDRIGHTLEGKINEMATICS
%    [T,J] = GENERATEDRIGHTLEGKINEMATICS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    27-Feb-2018 01:00:05

q_j3 = in2(1,:);
q_j4 = in2(2,:);
q_m6 = in1(1,:);
q_m7 = in1(2,:);
q_m8 = in1(3,:);
q_m9 = in1(4,:);
q_m10 = in1(5,:);
t2 = q_j3+q_j4+q_m8+q_m9+q_m10;
t3 = cos(q_m7);
t4 = sin(t2);
t5 = cos(t2);
t6 = sin(q_m7);
t7 = q_j3+q_m8+q_m9;
t8 = cos(t7);
t9 = cos(q_j4);
t10 = sin(t7);
t11 = sin(q_j4);
t12 = q_j3+q_j4+q_m8+q_m9;
t13 = cos(t12);
t14 = cos(q_m10);
t15 = sin(t12);
t16 = sin(q_m10);
t17 = q_m8+q_m9;
t18 = cos(t17);
t19 = cos(q_j3);
t20 = sin(t17);
t21 = sin(q_j3);
t22 = cos(q_m8);
t23 = cos(q_m9);
t24 = sin(q_m8);
t25 = sin(q_m9);
t26 = sin(q_m6);
t27 = cos(q_m6);
t28 = t22.*t26;
t29 = t6.*t24.*t27;
t30 = t28+t29;
t31 = t24.*t26;
t34 = t6.*t22.*t27;
t32 = t31-t34;
t33 = t23.*t30;
t39 = t25.*t32;
t35 = t33-t39;
t36 = t23.*t32;
t37 = t25.*t30;
t38 = t36+t37;
t40 = t21.*t35;
t41 = t19.*t38;
t42 = t40+t41;
t43 = t19.*t35;
t45 = t21.*t38;
t44 = t43-t45;
t46 = t9.*t44;
t49 = t11.*t42;
t47 = t46-t49;
t48 = t9.*t42;
t50 = t11.*(t43-t45);
t51 = t48+t50;
t52 = t22.*t27;
t57 = t6.*t24.*t26;
t53 = t52-t57;
t54 = t24.*t27;
t55 = t6.*t22.*t26;
t56 = t54+t55;
t58 = t23.*t53;
t63 = t25.*t56;
t59 = t58-t63;
t60 = t23.*t56;
t61 = t25.*t53;
t62 = t60+t61;
t64 = t21.*t59;
t65 = t19.*t62;
t66 = t64+t65;
t67 = t19.*t59;
t71 = t21.*t62;
t68 = t67-t71;
t69 = t9.*t68;
t70 = t9.*t66;
t72 = t11.*t68;
t73 = t70+t72;
t75 = t11.*t66;
t74 = t69-t75;
t76 = t11.*(t67-t71);
t77 = t70+t76;
t78 = t6.*9.999999999999999e-1;
t79 = 5.124451190127583e1;
t80 = atan(5.0./5.1e1);
t81 = q_j3+q_j4+q_m8+q_m9-t80;
t82 = cos(t81);
t83 = t79.*t82.*8.0e2;
t84 = 1.088049452001149e4;
t85 = atan(4.600239212439047e-2);
t86 = q_j3+q_m8+q_m9+t85;
t87 = cos(t86);
t88 = t84.*t87.*4.0;
t89 = 5.50841220316708e3;
t90 = atan(3.376125694577505e-1);
t91 = q_j3+q_j4+q_m8+q_m9+q_m10-t90;
t92 = sin(t91);
t93 = 7.700500308421525e3;
t94 = atan(7.813117996044825e-1);
t95 = q_m8+q_m9+t94;
t96 = cos(t95);
t97 = t93.*t96;
t98 = t21.*t62.*4.3476e-1;
t99 = t21.*t59.*(1.0./5.0e1);
t100 = t3.*t26.*4.5e-3;
t101 = t11.*t66.*(5.1e1./1.25e2);
t102 = t23.*t56.*4.741e-2;
t103 = t25.*t56.*6.068e-2;
t104 = t25.*t53.*4.741e-2;
t105 = t14.*t77.*5.219e-2;
t106 = t16.*t77.*1.762e-2;
t107 = t19.*t62.*(1.0./5.0e1);
t108 = t6.*t24.*t26.*(3.0./2.5e1);
t109 = t21.*t38.*(1.0./5.0e1);
t110 = t9.*(t43-t45).*(1.0./2.5e1);
t111 = t25.*t32.*4.741e-2;
t112 = t16.*t51.*5.219e-2;
t113 = t26.*(9.0./1.0e2);
t114 = t3.*t27.*4.5e-3;
t115 = t22.*t26.*(3.0./2.5e1);
t116 = t9.*t42.*(1.0./2.5e1);
t117 = t9.*(t43-t45).*(5.1e1./1.25e2);
t118 = t11.*(t43-t45).*(1.0./2.5e1);
t119 = t23.*t30.*6.068e-2;
t120 = t14.*(t46-t49).*1.762e-2;
t121 = t19.*t35.*4.3476e-1;
t122 = t6.*t24.*t27.*(3.0./2.5e1);
T = reshape([t3.*(t4.*6.899914937159737e15-t5.*5.78971607892534e15).*(-1.110223024625157e-16),t14.*t47.*(-7.66044443118978e-1)-t16.*t47.*6.427876096865393e-1+t16.*t51.*7.66044443118978e-1-t14.*(t48+t11.*t44).*6.427876096865393e-1,t14.*t73.*6.427876096865393e-1+t14.*t74.*7.66044443118978e-1-t16.*t73.*7.66044443118978e-1+t16.*t74.*6.427876096865393e-1,0.0,-t6,t3.*t27,t3.*t26,0.0,t3.*(t4.*5.78971607892534e15+t5.*6.899914937159737e15).*(-1.110223024625157e-16),t14.*t47.*(-6.427876096865393e-1)+t14.*t51.*7.66044443118978e-1+t16.*t51.*6.427876096865393e-1+t16.*(t46-t49).*7.66044443118978e-1,t14.*t73.*(-7.66044443118978e-1)+t14.*t74.*6.427876096865393e-1-t16.*t73.*6.427876096865393e-1-t16.*t74.*7.66044443118978e-1,0.0,t6.*(-4.5e-3)+t3.*t24.*(3.0./2.5e1)-t3.*t8.*t9.*(1.0./2.5e1)+t3.*t8.*t11.*(5.1e1./1.25e2)+t3.*t9.*t10.*(5.1e1./1.25e2)+t3.*t10.*t11.*(1.0./2.5e1)+t3.*t13.*t14.*5.219e-2+t3.*t13.*t16.*1.762e-2+t3.*t14.*t15.*1.762e-2-t3.*t15.*t16.*5.219e-2+t3.*t18.*t19.*(1.0./5.0e1)+t3.*t18.*t21.*4.3476e-1+t3.*t19.*t20.*4.3476e-1-t3.*t20.*t21.*(1.0./5.0e1)+t3.*t22.*t23.*4.741e-2+t3.*t22.*t25.*6.068e-2+t3.*t23.*t24.*6.068e-2-t3.*t24.*t25.*4.741e-2-4.9e1./1.0e3,t113+t114+t115+t116+t117+t118+t119+t120+t121+t122-t11.*t42.*(5.1e1./1.25e2)-t23.*t32.*4.741e-2-t25.*t30.*4.741e-2-t21.*t35.*(1.0./5.0e1)-t19.*t38.*(1.0./5.0e1)-t25.*t32.*6.068e-2-t21.*t38.*4.3476e-1-t16.*t47.*5.219e-2-t14.*t51.*5.219e-2-t16.*t51.*1.762e-2-2.7e1./2.0e2,t27.*(-9.0./1.0e2)+t98+t99+t100+t101+t102+t103+t104+t105+t106+t107+t108-t22.*t27.*(3.0./2.5e1)-t9.*t66.*(1.0./2.5e1)-t23.*t53.*6.068e-2-t9.*t68.*(5.1e1./1.25e2)-t19.*t59.*4.3476e-1-t11.*t68.*(1.0./2.5e1)-t14.*t74.*1.762e-2+t16.*(t69-t75).*5.219e-2,1.0],[4,4]);
if nargout > 1
    t123 = t3.*t24.*1.2e4;
    t124 = t3.*t8.*t11.*4.08e4;
    t125 = t3.*t9.*t10.*4.08e4;
    t126 = t3.*t10.*t11.*4.0e3;
    t127 = t3.*t13.*t14.*5.219e3;
    t128 = t3.*t13.*t16.*1.762e3;
    t129 = t3.*t14.*t15.*1.762e3;
    t130 = t3.*t18.*t19.*2.0e3;
    t131 = t3.*t18.*t21.*4.3476e4;
    t132 = t3.*t19.*t20.*4.3476e4;
    t133 = t3.*t22.*t23.*4.741e3;
    t134 = t3.*t22.*t25.*6.068e3;
    t135 = t3.*t23.*t24.*6.068e3;
    t136 = t6.*-4.5e2+t123+t124+t125+t126+t127+t128+t129+t130+t131+t132+t133+t134+t135-t3.*t8.*t9.*4.0e3-t3.*t15.*t16.*5.219e3-t3.*t20.*t21.*2.0e3-t3.*t24.*t25.*4.741e3;
    t137 = t21.*t59.*4.3476e-1;
    t138 = t9.*t66.*(5.1e1./1.25e2);
    t139 = t11.*t66.*(1.0./2.5e1);
    t140 = t11.*(t67-t71).*(5.1e1./1.25e2);
    t141 = t23.*t56.*6.068e-2;
    t142 = t23.*t53.*4.741e-2;
    t143 = t25.*t53.*6.068e-2;
    t144 = t14.*(t69-t75).*5.219e-2;
    t145 = t14.*t77.*1.762e-2;
    t146 = t16.*(t69-t75).*1.762e-2;
    t147 = t19.*t62.*4.3476e-1;
    t148 = t19.*t59.*(1.0./5.0e1);
    J = reshape([cos(q_m7.*2.0).*3.225113306814322e-17+9.999999999999999e-1,t3.*t6.*t27.*6.450226613628643e-17,t3.*t6.*t26.*6.450226613628643e-17,0.0,t27.*(9.0./1.0e2)-t98-t99-t100-t101-t102-t103-t104-t105-t106-t107-t108+t22.*t27.*(3.0./2.5e1)+t9.*t66.*(1.0./2.5e1)+t23.*t53.*6.068e-2+t19.*t59.*4.3476e-1-t16.*t74.*5.219e-2+t9.*(t67-t71).*(5.1e1./1.25e2)+t11.*(t67-t71).*(1.0./2.5e1)+t14.*(t69-t75).*1.762e-2,t113+t114+t115+t116+t117+t118+t119+t120+t121+t122-t11.*t42.*(5.1e1./1.25e2)-t23.*t32.*4.741e-2-t25.*t30.*4.741e-2-t21.*t35.*(1.0./5.0e1)-t19.*t38.*(1.0./5.0e1)-t25.*t32.*6.068e-2-t21.*t38.*4.3476e-1-t16.*t47.*5.219e-2-t14.*t51.*5.219e-2-t16.*t51.*1.762e-2,0.0,t26.*(-9.999999999999999e-1),t27.*9.999999999999999e-1,t3.*(-4.5e-3)-t6.*t24.*(3.0./2.5e1)+t6.*t8.*t9.*(1.0./2.5e1)-t6.*t8.*t11.*(5.1e1./1.25e2)-t6.*t9.*t10.*(5.1e1./1.25e2)-t6.*t10.*t11.*(1.0./2.5e1)-t6.*t13.*t14.*5.219e-2-t6.*t13.*t16.*1.762e-2-t6.*t14.*t15.*1.762e-2+t6.*t15.*t16.*5.219e-2-t6.*t18.*t19.*(1.0./5.0e1)-t6.*t18.*t21.*4.3476e-1-t6.*t19.*t20.*4.3476e-1+t6.*t20.*t21.*(1.0./5.0e1)-t6.*t22.*t23.*4.741e-2-t6.*t22.*t25.*6.068e-2-t6.*t23.*t24.*6.068e-2+t6.*t24.*t25.*4.741e-2,t27.*t136.*1.0e-5,t26.*t136.*1.0e-5,t78,t3.*t27.*(-9.999999999999999e-1),t3.*t26.*(-9.999999999999999e-1),t3.*(t22.*1.2e4+t83+t88+t97-t89.*t92).*1.0e-5,t109+t110+t111+t112-t24.*t26.*(3.0./2.5e1)-t9.*t42.*(5.1e1./1.25e2)-t11.*t42.*(1.0./2.5e1)-t23.*t30.*4.741e-2-t19.*t35.*(1.0./5.0e1)-t11.*t44.*(5.1e1./1.25e2)-t23.*t32.*6.068e-2-t25.*t30.*6.068e-2-t21.*t35.*4.3476e-1-t19.*t38.*4.3476e-1-t14.*t47.*5.219e-2-t16.*t47.*1.762e-2-t14.*t51.*1.762e-2+t6.*t22.*t27.*(3.0./2.5e1),t137+t138+t139+t140+t141+t142+t143+t144+t145+t146+t147+t148+t24.*t27.*(3.0./2.5e1)-t9.*t68.*(1.0./2.5e1)-t25.*t56.*4.741e-2-t21.*t62.*(1.0./5.0e1)-t16.*t77.*5.219e-2+t6.*t22.*t26.*(3.0./2.5e1),t78,t3.*t27.*(-9.999999999999999e-1),t3.*t26.*(-9.999999999999999e-1),t3.*(t83+t88+t97-t89.*t92).*1.0e-5,t109+t110+t111+t112-t9.*t42.*(5.1e1./1.25e2)-t11.*t42.*(1.0./2.5e1)-t23.*t30.*4.741e-2-t19.*t35.*(1.0./5.0e1)-t11.*t44.*(5.1e1./1.25e2)-t23.*t32.*6.068e-2-t25.*t30.*6.068e-2-t21.*t35.*4.3476e-1-t19.*t38.*4.3476e-1-t14.*t47.*5.219e-2-t16.*t47.*1.762e-2-t14.*t51.*1.762e-2,t137+t138+t139+t140+t141+t142+t143+t144+t145+t146+t147+t148-t9.*t68.*(1.0./2.5e1)-t25.*t56.*4.741e-2-t21.*t62.*(1.0./5.0e1)-t16.*t77.*5.219e-2,t78,t3.*t27.*(-9.999999999999999e-1),t3.*t26.*(-9.999999999999999e-1),t3.*t89.*t92.*(-1.0e-5),t112-t14.*t47.*5.219e-2-t16.*t47.*1.762e-2-t14.*t51.*1.762e-2,t144+t145+t146-t16.*t77.*5.219e-2,t78,t3.*t27.*(-9.999999999999999e-1),t3.*t26.*(-9.999999999999999e-1),t3.*(t83+t88-t89.*t92).*1.0e-5,t109+t110+t112-t9.*t42.*(5.1e1./1.25e2)-t11.*t42.*(1.0./2.5e1)-t19.*t35.*(1.0./5.0e1)-t11.*t44.*(5.1e1./1.25e2)-t21.*t35.*4.3476e-1-t19.*t38.*4.3476e-1-t14.*t47.*5.219e-2-t16.*t47.*1.762e-2-t14.*t51.*1.762e-2,t137+t138+t139+t140+t144+t145+t146+t147+t148-t9.*t68.*(1.0./2.5e1)-t21.*t62.*(1.0./5.0e1)-t16.*t77.*5.219e-2,t78,t3.*t27.*(-9.999999999999999e-1),t3.*t26.*(-9.999999999999999e-1),t3.*(t83-t89.*t92).*1.0e-5,t110+t112-t9.*t42.*(5.1e1./1.25e2)-t11.*t42.*(1.0./2.5e1)-t11.*t44.*(5.1e1./1.25e2)-t14.*t47.*5.219e-2-t16.*t47.*1.762e-2-t14.*t51.*1.762e-2,t138+t139+t140+t144+t145+t146-t9.*t68.*(1.0./2.5e1)-t16.*t77.*5.219e-2],[6,7]);
end
