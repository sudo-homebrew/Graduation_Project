function [hSolns, hasFiniteNumSol, hasPiSoln] = solveForHGeneralCase(R3, z3, a1, a2, a3, alpha1, alpha2, alpha3, d2, d3, d4)
%solveForHGeneralCase Solve for h when a1 and alpha1 are nonzero
%   To solve for theta3, it is necessary to reparameterize a trigonometric
%   equation in terms of a new parameter h = tan(that3/2) using the
%   Weierstrass equation. This function solves equation 3.39 in the Pieper
%   inverse kinematics solution for h in the case where DH parameters a1
%   and alpha1 are both nonzero. The equation arises from the sum of
%   squares of the following two equations:
%      3.25: R3 = F1cos(theta2) + F2sin(theta2)2a1 + F3
%      3.26: z3 = F1sin(theta2) - F2cos(theta2)sin(alpha1) + F4
%   Here F1 to F4 are functions of theta3 (and the constant DH parameters),
%   and R3 and z3 are functions of P, a known position input from the IK
%   problem, and DH parameter d1:
%      R3 = P(1)^2 + P(2)^2 + (P(3) – d1)^2 
%      z3 = P(3) - d1
%   The sum of squares produces a single equation that may be
%   reparameterized in h, producing a quartic polynomial in h. This
%   function solves that polynomial for the values of h given R3, z3, and
%   the DH parameters of the associated serial manipulator.
 
%   Copyright 2020 The MathWorks, Inc.
 
% Compute the polynomial coefficients
[A,B,C,D,E] = getQuarticPolynomialCoeffs(R3, z3, ...
    a1, a2, a3, ...
    alpha1, alpha2, alpha3, ...
    d2, d3, d4   ...
    );

% Add exception to handle the trivial case
if isEqualWithinTolerance([A B C D E], [0 0 0 0 0])
    % The trivial case happens when the rotation of theta3 has no impact on
    % the end effector position (only on the orientation) because the next
    % joint lies on the axis of rotation. Since this equation is derived
    % from the position solution, any real-valued orientation solution
    % would work. Default to zero.
    hSolns = 0;
    hasFiniteNumSol = false;
    hasPiSoln = true;
else
    % Solve polynomial. While there are four solutions to the quartic, there
    % can be at most two solutions for this variable -- the others are false
    % solutions that arise from the sum of squares. These will be eliminated
    % below by using constraint equations.
    hSolns = solveQuarticPolynomial([A B C D E]);
    hasFiniteNumSol = true;
    
    fTerms = computef13SupportingEquations(a3, alpha3, d3, d4, pi);
    FTerms = computeF14SupportingEquations(a1, a2, alpha1, alpha2, fTerms(1), fTerms(2), fTerms(3), d2);
    
    % Check if there is a solution at theta3 = pi, for which h is undefined, by
    % checking if R3 = F3 (eq 3.25) is satisfied for that solution.
    eq339LHS = (FTerms(4) - z3)^2/sin(alpha1)^2 + (FTerms(3) - R3)^2/(4*a1^2);
    eq339RHS = FTerms(1)^2 + FTerms(2)^2;
    hasPiSoln = isEqualWithinTolerance(eq339LHS, eq339RHS);
end
 
    % Helper functions
    function [h4Coef, h3Coef, h2Coef, h1Coef, h0Coef] = getQuarticPolynomialCoeffs(R3s,z3s,a1,a2,a3,alpha1,alpha2,alpha3,d2,d3,d4)
        %getQuarticPolynomialCoeffs Compute the coefficients of the quartic polynomial
        %   The first part of the solution is a fourth-order polynomial in h =
        %   tan(theta3/2). This function computes the coefficients of that
        %   polynomial, A*h^4 + B*h^3 + C*h^2 + D*h + E = 0.
                
         %Polynomial coefficients for DH robot
        t2 = cos(alpha1);
        t3 = cos(alpha2);
        t4 = cos(alpha3);
        t5 = sin(alpha1);
        t6 = sin(alpha2);
        t7 = sin(alpha3);
        t8 = R3s.^2;
        t9 = a1.^2;
        t10 = a2.^2;
        t12 = a2.^3;
        t13 = a3.^2;
        t15 = a3.^3;
        t17 = d2.^2;
        t18 = d2.^3;
        t19 = d3.^2;
        t21 = d3.^3;
        t22 = d4.^2;
        t24 = d4.^3;
        t26 = z3s.^2;
        t11 = t9.^2;
        t14 = t10.^2;
        t16 = t13.^2;
        t20 = t17.^2;
        t23 = t19.^2;
        t25 = t22.^2;
        t27 = t2.^2;
        t28 = t3.^2;
        t29 = t4.^2;
        t30 = t4.^3;
        t32 = t5.^2;
        t33 = t6.^2;
        t34 = t7.^2;
        t35 = t7.^3;
        t43 = t9.*t26.*4.0;
        t52 = d2.*t2.*t9.*z3s.*8.0;
        t80 = a3.*t2.*t6.*t9.*z3s.*1.6e+1;
        t81 = d3.*t2.*t3.*t9.*z3s.*8.0;
        t110 = d4.*t2.*t3.*t4.*t9.*z3s.*8.0;
        t112 = d4.*t2.*t6.*t7.*t9.*z3s.*8.0;
        t31 = t29.^2;
        t36 = t34.^2;
        t37 = t8.*t32;
        t38 = t11.*t32;
        t39 = t14.*t32;
        t40 = t16.*t32;
        t41 = t20.*t32;
        t42 = t23.*t32;
        t44 = R3s.*a2.*a3.*t32.*4.0;
        t45 = R3s.*t9.*t32.*2.0;
        t46 = R3s.*t10.*t32.*2.0;
        t47 = R3s.*t13.*t32.*2.0;
        t48 = a2.*t15.*t32.*4.0;
        t49 = a3.*t12.*t32.*4.0;
        t50 = R3s.*t17.*t32.*2.0;
        t51 = R3s.*t19.*t32.*2.0;
        t53 = a2.*a3.*t9.*t32.*4.0;
        t54 = a2.*a3.*t17.*t32.*4.0;
        t55 = a2.*a3.*t19.*t32.*4.0;
        t56 = R3s.*d2.*d3.*t3.*t32.*4.0;
        t57 = R3s.*d3.*d4.*t4.*t32.*4.0;
        t63 = -t52;
        t64 = t9.*t10.*t32.*2.0;
        t65 = t9.*t13.*t32.*2.0;
        t66 = t10.*t13.*t32.*6.0;
        t67 = t9.*t17.*t27.*4.0;
        t68 = t9.*t17.*t32.*2.0;
        t69 = t9.*t19.*t32.*2.0;
        t70 = t10.*t17.*t32.*2.0;
        t71 = t10.*t19.*t32.*2.0;
        t72 = t13.*t17.*t32.*2.0;
        t73 = t13.*t19.*t32.*2.0;
        t74 = t17.*t19.*t32.*2.0;
        t76 = R3s.*a3.*d2.*t6.*t32.*8.0;
        t77 = R3s.*a2.*d4.*t7.*t32.*8.0;
        t83 = d2.*t3.*t21.*t32.*4.0;
        t84 = d3.*t3.*t18.*t32.*4.0;
        t85 = d4.*t4.*t21.*t32.*4.0;
        t86 = a2.*a3.*d2.*d3.*t3.*t32.*8.0;
        t87 = a2.*a3.*d3.*d4.*t4.*t32.*8.0;
        t88 = d3.*d4.*t4.*t17.*t32.*4.0;
        t91 = R3s.*d2.*d4.*t3.*t4.*t32.*4.0;
        t92 = R3s.*d2.*d4.*t6.*t7.*t32.*4.0;
        t95 = -t80;
        t96 = -t81;
        t97 = a3.*t6.*t18.*t32.*8.0;
        t98 = d2.*t6.*t15.*t32.*8.0;
        t99 = d4.*t7.*t12.*t32.*8.0;
        t100 = d2.*d3.*t3.*t9.*t32.*4.0;
        t101 = d2.*d3.*t3.*t10.*t32.*4.0;
        t102 = d2.*d3.*t3.*t13.*t32.*4.0;
        t103 = d3.*d4.*t4.*t9.*t32.*4.0;
        t104 = d3.*d4.*t4.*t10.*t32.*4.0;
        t105 = d3.*d4.*t4.*t13.*t32.*4.0;
        t106 = a3.*d2.*t6.*t19.*t32.*8.0;
        t107 = a2.*d4.*t7.*t17.*t32.*8.0;
        t108 = a2.*d4.*t7.*t19.*t32.*8.0;
        t111 = d4.*t3.*t4.*t18.*t32.*4.0;
        t113 = d4.*t6.*t7.*t18.*t32.*4.0;
        t114 = R3s.*t22.*t29.*t32.*2.0;
        t115 = R3s.*t22.*t32.*t34.*2.0;
        t116 = a3.*d2.*t6.*t9.*t27.*1.6e+1;
        t117 = d2.*d3.*t3.*t9.*t27.*8.0;
        t118 = a3.*d2.*t6.*t9.*t32.*8.0;
        t119 = a3.*d2.*t6.*t10.*t32.*8.0;
        t120 = a2.*d4.*t7.*t9.*t32.*8.0;
        t121 = a2.*d4.*t7.*t13.*t32.*8.0;
        t122 = a2.*d2.*t6.*t13.*t32.*1.6e+1;
        t123 = a3.*d4.*t7.*t9.*t32.*1.6e+1;
        t124 = a3.*d4.*t7.*t10.*t32.*1.6e+1;
        t125 = d3.*t24.*t30.*t32.*4.0;
        t126 = a2.*a3.*t22.*t29.*t32.*4.0;
        t127 = -t110;
        t128 = a2.*a3.*t22.*t32.*t34.*4.0;
        t129 = d2.*d4.*t3.*t4.*t9.*t32.*4.0;
        t130 = d2.*d4.*t3.*t4.*t10.*t32.*4.0;
        t131 = d2.*d4.*t3.*t4.*t13.*t32.*4.0;
        t132 = a2.*a3.*d2.*d4.*t3.*t4.*t32.*8.0;
        t134 = d2.*d4.*t6.*t7.*t9.*t32.*4.0;
        t135 = d2.*d4.*t6.*t7.*t10.*t32.*4.0;
        t136 = d2.*d4.*t6.*t7.*t13.*t32.*4.0;
        t137 = a2.*a3.*d2.*d4.*t6.*t7.*t32.*8.0;
        t138 = a2.*d2.*d3.*d4.*t3.*t7.*t32.*1.6e+1;
        t139 = a3.*d2.*d3.*d4.*t4.*t6.*t32.*1.6e+1;
        t141 = d2.*d4.*t6.*t7.*t19.*t32.*4.0;
        t142 = a2.*t24.*t32.*t35.*8.0;
        t144 = t9.*t19.*t27.*t28.*4.0;
        t145 = t9.*t22.*t29.*t32.*2.0;
        t146 = t10.*t22.*t29.*t32.*2.0;
        t147 = t13.*t22.*t29.*t32.*2.0;
        t148 = t9.*t19.*t32.*t33.*4.0;
        t149 = t9.*t22.*t32.*t34.*2.0;
        t150 = t10.*t22.*t32.*t34.*2.0;
        t151 = t13.*t22.*t32.*t34.*2.0;
        t152 = t17.*t19.*t28.*t32.*4.0;
        t153 = t17.*t22.*t29.*t32.*2.0;
        t154 = t19.*t22.*t29.*t32.*6.0;
        t155 = t17.*t22.*t32.*t34.*2.0;
        t156 = t19.*t22.*t32.*t34.*2.0;
        t157 = a3.*d3.*t3.*t6.*t9.*t27.*1.6e+1;
        t158 = d2.*d4.*t3.*t4.*t9.*t27.*8.0;
        t159 = a3.*d3.*t3.*t6.*t9.*t32.*1.6e+1;
        t160 = d2.*t3.*t24.*t30.*t32.*4.0;
        t161 = d3.*t4.*t24.*t32.*t34.*4.0;
        t162 = d2.*d4.*t6.*t7.*t9.*t27.*8.0;
        t163 = a3.*d3.*t3.*t6.*t17.*t32.*1.6e+1;
        t164 = a2.*d3.*t4.*t7.*t22.*t32.*1.6e+1;
        t165 = d2.*d4.*t3.*t4.*t19.*t32.*1.2e+1;
        t166 = d2.*t6.*t24.*t32.*t35.*4.0;
        t168 = d2.*d3.*t3.*t22.*t32.*t34.*4.0;
        t170 = t25.*t29.*t32.*t34.*2.0;
        t171 = a2.*t7.*t24.*t29.*t32.*8.0;
        t172 = d3.*d4.*t4.*t9.*t27.*t28.*8.0;
        t173 = a3.*d4.*t7.*t9.*t27.*t33.*1.6e+1;
        t175 = d3.*d4.*t4.*t9.*t32.*t33.*8.0;
        t176 = a3.*d2.*t6.*t22.*t29.*t32.*8.0;
        t177 = d3.*d4.*t4.*t17.*t28.*t32.*8.0;
        t178 = d2.*d3.*t3.*t22.*t29.*t32.*1.2e+1;
        t179 = a3.*d2.*t6.*t22.*t32.*t34.*8.0;
        t180 = a2.*d2.*t6.*t22.*t32.*t34.*1.6e+1;
        t181 = a3.*d4.*t7.*t17.*t32.*t33.*1.6e+1;
        t182 = a3.*d4.*t3.*t4.*t6.*t9.*t27.*1.6e+1;
        t183 = a3.*d4.*t3.*t4.*t6.*t9.*t32.*1.6e+1;
        t184 = d2.*t3.*t4.*t24.*t32.*t34.*4.0;
        t185 = d3.*d4.*t3.*t6.*t7.*t9.*t27.*8.0;
        t186 = a2.*d2.*t3.*t4.*t7.*t22.*t32.*1.6e+1;
        t187 = a3.*d4.*t3.*t4.*t6.*t17.*t32.*1.6e+1;
        t188 = d2.*t6.*t7.*t24.*t29.*t32.*4.0;
        t189 = d3.*d4.*t3.*t6.*t7.*t9.*t32.*8.0;
        t190 = d3.*d4.*t3.*t6.*t7.*t17.*t32.*8.0;
        t191 = d2.*d3.*t4.*t6.*t7.*t22.*t32.*8.0;
        t193 = t9.*t22.*t27.*t28.*t29.*4.0;
        t194 = t9.*t22.*t27.*t33.*t34.*4.0;
        t195 = t9.*t22.*t28.*t32.*t34.*4.0;
        t196 = t9.*t22.*t29.*t32.*t33.*4.0;
        t197 = t17.*t22.*t28.*t29.*t32.*4.0;
        t198 = t17.*t22.*t32.*t33.*t34.*4.0;
        t199 = t3.*t4.*t6.*t7.*t9.*t22.*t27.*8.0;
        t200 = t3.*t4.*t6.*t7.*t9.*t22.*t32.*8.0;
        t201 = t3.*t4.*t6.*t7.*t17.*t22.*t32.*8.0;
        t58 = -t45;
        t59 = -t46;
        t60 = -t47;
        t61 = -t50;
        t62 = -t51;
        t75 = t25.*t31.*t32;
        t78 = -t56;
        t79 = -t57;
        t82 = t25.*t32.*t36;
        t89 = -t64;
        t90 = -t65;
        t93 = -t76;
        t94 = -t77;
        t109 = -t91;
        t133 = -t114;
        t140 = -t115;
        t143 = -t120;
        t167 = -t137;
        t169 = -t148;
        t174 = t28.*t123;
        t192 = -t175;
        t202 = -t195;
        t203 = -t196;
        
        h4Coef = t37+t38+t39+t40+t41+t42+t43+t44-t48-t49+t53-t54-t55+t58+t59+t60+t61+t62+t63+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t78+t79+t82+t83+t84+t85-t86-t87+t88+t89+t90-t92+t96+t100+t101+t102+t103+t104+t105+t109+t111-t112+t113+t117+t125-t126+t127-t128+t129+t130+t131-t132+t133+t134+t135+t136+t140+t141+t144+t145+t146+t147+t149+t150+t151+t152+t153+t154+t155+t156+t158+t160+t161+t162+t165+t166+t167+t168+t169+t170+t172+t177+t178+t184+t185+t188+t189+t190+t191+t192+t193+t194+t197+t198+t199+t200+t201+t202+t203;
        if nargout > 1
            h3Coef = t93+t94+t95+t97+t98+t99+t106+t107+t108+t116+t118+t119+t121-t122+t123-t124+t138+t139+t142+t143+t157+t159+t163+t164+t171+t173+t176+t179+t180+t181+t182+t183+t186+t187-a3.*d4.*t7.*t9.*t28.*t32.*1.6e+1;
        end
        if nargout > 2
            h2Coef = t37.*2.0+t38.*2.0+t39.*2.0+t40.*2.0+t41.*2.0+t42.*2.0+t75.*2.0+t82.*2.0+t9.*t26.*8.0-R3s.*t9.*t32.*4.0-R3s.*t10.*t32.*4.0-R3s.*t13.*t32.*4.0-R3s.*t17.*t32.*4.0-R3s.*t19.*t32.*4.0-t9.*t10.*t32.*4.0+t9.*t17.*t27.*8.0+t9.*t13.*t32.*1.2e+1-t10.*t13.*t32.*4.0+t9.*t17.*t32.*4.0+t10.*t17.*t32.*4.0+t9.*t19.*t32.*4.0+t10.*t19.*t32.*4.0+t13.*t17.*t32.*4.0+t13.*t19.*t32.*4.0+t17.*t19.*t32.*4.0-R3s.*t22.*t29.*t32.*4.0-R3s.*t22.*t32.*t34.*4.0+d3.*t3.*t18.*t32.*8.0+d2.*t3.*t21.*t32.*8.0+d4.*t4.*t21.*t32.*8.0+d3.*t24.*t30.*t32.*8.0+t9.*t13.*t27.*t33.*1.6e+1-t9.*t13.*t28.*t32.*1.6e+1+t9.*t19.*t27.*t28.*8.0+t9.*t22.*t29.*t32.*4.0-t9.*t19.*t32.*t33.*8.0+t10.*t22.*t29.*t32.*4.0+t13.*t17.*t32.*t33.*1.6e+1+t13.*t22.*t29.*t32.*4.0+t17.*t19.*t28.*t32.*8.0-t9.*t22.*t32.*t34.*1.2e+1+t10.*t22.*t32.*t34.*2.0e+1+t17.*t22.*t29.*t32.*4.0+t13.*t22.*t32.*t34.*4.0+t19.*t22.*t29.*t32.*1.2e+1+t17.*t22.*t32.*t34.*4.0+t19.*t22.*t32.*t34.*4.0+t25.*t29.*t32.*t34.*4.0-d2.*t2.*t9.*z3s.*1.6e+1-R3s.*d2.*d3.*t3.*t32.*8.0-R3s.*d3.*d4.*t4.*t32.*8.0+d2.*d3.*t3.*t9.*t27.*1.6e+1+d2.*d3.*t3.*t9.*t32.*8.0+d2.*d3.*t3.*t10.*t32.*8.0+d3.*d4.*t4.*t9.*t32.*8.0+d2.*d3.*t3.*t13.*t32.*8.0+d3.*d4.*t4.*t10.*t32.*8.0+d3.*d4.*t4.*t13.*t32.*8.0+d3.*d4.*t4.*t17.*t32.*8.0+d4.*t3.*t4.*t18.*t32.*8.0+d2.*t3.*t24.*t30.*t32.*8.0+d3.*t4.*t24.*t32.*t34.*8.0+t9.*t22.*t27.*t28.*t29.*8.0-t9.*t22.*t27.*t33.*t34.*8.0+t9.*t22.*t28.*t32.*t34.*8.0-t9.*t22.*t29.*t32.*t33.*8.0+t17.*t22.*t28.*t29.*t32.*8.0-t17.*t22.*t32.*t33.*t34.*8.0-d3.*t2.*t3.*t9.*z3s.*1.6e+1+d2.*d4.*t3.*t4.*t9.*t27.*1.6e+1+d2.*d4.*t3.*t4.*t9.*t32.*8.0+d2.*d4.*t3.*t4.*t10.*t32.*8.0+d2.*d4.*t3.*t4.*t13.*t32.*8.0+d2.*d4.*t3.*t4.*t19.*t32.*2.4e+1+d3.*d4.*t4.*t9.*t27.*t28.*1.6e+1-d3.*d4.*t4.*t9.*t32.*t33.*1.6e+1+d3.*d4.*t4.*t17.*t28.*t32.*1.6e+1+d2.*d3.*t3.*t22.*t29.*t32.*2.4e+1+d2.*d3.*t3.*t22.*t32.*t34.*8.0+d2.*t3.*t4.*t24.*t32.*t34.*8.0-d4.*t2.*t3.*t4.*t9.*z3s.*1.6e+1-R3s.*d2.*d4.*t3.*t4.*t32.*8.0+a2.*a3.*d2.*d4.*t6.*t7.*t32.*4.8e+1;
        end
        if nargout > 3
            h1Coef = t93+t94+t95+t97+t98+t99+t106+t107+t108+t116+t118+t119+t121+t122-t123+t124+t138+t139+t142+t143+t157+t159+t163+t164+t171-t173+t174+t176+t179-t180-t181+t182+t183+t186+t187;
        end
        if nargout > 4
            h0Coef = t37+t38+t39+t40+t41+t42+t43-t44+t48+t49-t53+t54+t55+t58+t59+t60+t61+t62+t63+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t78+t79+t82+t83+t84+t85+t86+t87+t88+t89+t90+t92+t96+t100+t101+t102+t103+t104+t105+t109+t111+t112-t113+t117+t125+t126+t127+t128+t129+t130+t131+t132+t133-t134-t135-t136+t140-t141+t144+t145+t146+t147+t149+t150+t151+t152+t153+t154+t155+t156+t158+t160+t161-t162+t165-t166+t167+t168+t169+t170+t172+t177+t178+t184-t185-t188-t189-t190-t191+t192+t193+t194+t197+t198-t199-t200-t201+t202+t203;
        end
    end
end


