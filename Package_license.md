---
pagetitle: Licensed Materials Terms for the x-cube-n6-ai-multi-pose-estimation Application
lang: en
header-includes: <link rel="icon" type="image/x-icon" href="_htmresc/favicon.png" />
---

::: {.row}
::: {.col-sm-12 .col-lg-3}

<center>
# These are full license terms for
# STM32Cube.AI Application Examples:
# <mark>x-cube-n6-ai-multi-pose-estimation Application</mark>

Copyright &copy;2024 STMicroelectronics

All rights reserved

[![ST logo](_htmresc/st_logo_2020.png)](https://www.st.com){.logo}
</center>
:::

::: {.col-sm-12 .col-lg-9}

## <mark>__OVERVIEW__</mark>
<div>

This Software Bill Of Material (SBOM) lists the software components of this
software package, including the copyright owner and license terms for each
component.

The full text of these licenses are below the SBOM.

__SOFTWARE BILL OF MATERIALS__

| Component                                           | Copyright                             | License
|---------                                            |---------                              |-------
| Lib/Camera_Middleware/sensors                       | STMicroelectronics                    | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| Lib/Camera_Middleware                               | STMicroelectronics                    | [SLA](#collapse-section1)
| Lib/AI_Runtime                                      | STMicroelectronics                    | [SLA0104](#collapse-section3)
| Lib/Camera_Middleware/ISP_Library                   | LACROIX - Impulse, STMicroelectronics | [SLA](#collapse-section1)
| Lib/lib_vision_models_pp/lib_vision_models_pp       | STMicroelectronics                    | [SLA](#collapse-section1)
| EWARM/Src                                           | STMicroelectronics                    | [SLA](#collapse-section1)
| Gcc                                                 | STMicroelectronics                    | [SLA](#collapse-section1)
| Inc                                                 | STMicroelectronics                    | [SLA](#collapse-section1)
| Src                                                 | STMicroelectronics                    | [SLA](#collapse-section1)
| Model                                               | STMicroelectronics                    | [SLA](#collapse-section1)
| CMSIS                                               | ARM Limited                           | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32N6xx CMSIS Device                              | ARM Limited, STMicroelectronics       | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32N6xx HAL/LL Drivers                            | STMicroelectronics                    | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| STM32N6570-DK BSP Drivers                           | STMicroelectronics                    | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP Component aps256xx                              | STMicroelectronics                    | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP Component Common                                | STMicroelectronics                    | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP Component mx66uw1g45g                           | STMicroelectronics                    | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP Component rk050hr18                             | STMicroelectronics                    | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| Azure RTOS ThreadX                                  | Microsoft Corporation                 | [ANNEX 1](#collapse-section2)
| CMSIS RTOS Threadx wrapper                          | STMicroelectronics                    | [SLA](#collapse-section1)
| Binary                                              | STMicroelectronics, Ultralytics       | [SLA0078](#collapse-section4)

__Notes:__ If the license is an open source license, then you can access the
terms at [www.opensource.org](https://opensource.org/). Otherwise, the full
license terms are below. If a component is not listed in the SBOM, then the SLA
shall apply unless other terms are clearly stated in the package.

</div>


::: {.collapse}
<input type="checkbox" id="collapse-section1" checked aria-hidden="true">
<label for="collapse-section1" aria-hidden="true">__SLA0048__ – __Software License Agreement__</label>
<div>

SLA0048 Rev4/March 2018

## Software license agreement

### __SOFTWARE PACKAGE LICENSE AGREEMENT__

BY INSTALLING COPYING, DOWNLOADING, ACCESSING OR OTHERWISE USING THIS SOFTWARE PACKAGE OR ANY
PART THEREOF (AND THE RELATED DOCUMENTATION) FROM STMICROELECTRONICS INTERNATIONAL N.V, SWISS
BRANCH AND/OR ITS AFFILIATED COMPANIES (STMICROELECTRONICS), THE RECIPIENT, ON BEHALF OF HIMSELF
OR HERSELF, OR ON BEHALF OF ANY ENTITY BY WHICH SUCH RECIPIENT IS EMPLOYED AND/OR ENGAGED
AGREES TO BE BOUND BY THIS SOFTWARE PACKAGE LICENSE AGREEMENT.

Under STMicroelectronics’ intellectual property rights and subject to applicable licensing terms for any third-party software
incorporated in this software package and applicable Open Source Terms (as defined here below), the redistribution,
reproduction and use in source and binary forms of the software package or any part thereof, with or without modification, are
permitted provided that the following conditions are met:

1. Redistribution of source code (modified or not) must retain any copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form, except as embedded into microcontroller or microprocessor device manufactured by or for
STMicroelectronics or a software update for such device, must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of STMicroelectronics nor the names of other contributors to this software package may be used to
endorse or promote products derived from this software package or part thereof without specific written permission.

4. This software package or any part thereof, including modifications and/or derivative works of this software package, must
be used and execute solely and exclusively on or in combination with a microcontroller or a microprocessor devices
manufactured by or for STMicroelectronics.

5. No use, reproduction or redistribution of this software package partially or totally may be done in any manner that would
subject this software package to any Open Source Terms (as defined below).

6. Some portion of the software package may contain software subject to Open Source Terms (as defined below) applicable
for each such portion (“Open Source Software”), as further specified in the software package. Such Open Source Software
is supplied under the applicable Open Source Terms and is not subject to the terms and conditions of license hereunder.
“Open Source Terms” shall mean any open source license which requires as part of distribution of software that the source
code of such software is distributed therewith or otherwise made available, or open source license that substantially
complies with the Open Source definition specified at www.opensource.org and any other comparable open source license
such as for example GNU General Public License (GPL), Eclipse Public License (EPL), Apache Software License, BSD
license and MIT license.

7. This software package may also include third party software as expressly specified in the software package subject to
specific license terms from such third parties. Such third party software is supplied under such specific license terms and is
not subject to the terms and conditions of license hereunder. By installing copying, downloading, accessing or otherwise
using this software package, the recipient agrees to be bound by such license terms with regard to such third party
software.

8. STMicroelectronics has no obligation to provide any maintenance, support or updates for the software package.

9. The software package is and will remain the exclusive property of STMicroelectronics and its licensors. The recipient will
not take any action that jeopardizes STMicroelectronics and its licensors' proprietary rights or acquire any rights in the
software package, except the limited rights specified hereunder.

10. The recipient shall comply with all applicable laws and regulations affecting the use of the software package or any part
thereof including any applicable export control law or regulation.

11. Redistribution and use of this software package partially or any part thereof other than as permitted under this license is
void and will automatically terminate your rights under this license.

THIS SOFTWARE PACKAGE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY
INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE PACKAGE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

EXCEPT AS EXPRESSLY PERMITTED HEREUNDER AND SUBJECT TO THE APPLICABLE LICENSING TERMS FOR ANY
THIRD-PARTY SOFTWARE INCORPORATED IN THE SOFTWARE PACKAGE AND OPEN SOURCE TERMS AS
APPLICABLE, NO LICENSE OR OTHER RIGHTS, WHETHER EXPRESS OR IMPLIED, ARE GRANTED UNDER ANY
PATENT OR OTHER INTELLECTUAL PROPERTY RIGHTS OF STMICROELECTRONICS OR ANY THIRD PARTY.

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section3" checked aria-hidden="true">
<label for="collapse-section3" aria-hidden="true">__SLA0104__ – __Software License Agreement__</label>
<div>

SLA0104 Rev1/June 2024

## Software license agreement

### __SOFTWARE PACKAGE LICENSE AGREEMENT (“AGREEMENT”)__

Please indicate your acceptance or NON-acceptance by selecting “I ACCEPT” or “I DO NOT ACCEPT” as indicated below in the media.

BY INSTALLING COPYING, DOWNLOADING, ACCESSING OR OTHERWISE USING THIS SOFTWARE PACKAGE OR ANY PART THEREOF, INCLUDING ANY RELATED DOCUMENTATION (collectively the “SOFTWARE PACKAGE”) FROM STMICROELECTRONICS INTERNATIONAL N.V, SWISS BRANCH AND/OR ITS AFFILIATED COMPANIES (collectively "STMICROELECTRONICS”), YOU (hereinafter referred also to as “THE RECIPIENT”), ON BEHALF OF YOURSELF, OR ON BEHALF OF ANY ENTITY BY WHICH YOU ARE EMPLOYED AND/OR ENGAGED, AGREE TO BE BOUND BY THIS
AGREEMENT.

DOCUMENTATION shall mean and include any comments, annotations, instructions, manuals, and other materials, whether in
printed or electronic form, including without limitation installation manuals, user’s guides, and programmer guides, related to any
software made available under this AGREEMENT.

----------------------------------------

Subject to STMICROELECTRONICS’ intellectual property rights and applicable licensing terms for any third-party software incorporated in the SOFTWARE PACKAGE and applicable Open Source Terms (as defined here below), the redistribution, reproduction and use in source and binary forms of the SOFTWARE PACKAGE or any part thereof, with or without modification, are permitted provided that the following conditions are met:

1. Redistribution of source code (modified or not) must retain any copyright notice accompanying the SOFTWARE PACKAGE,

2. Redistributions in binary form, except as embedded into a device manufactured by or for STMicroelectronics or a software update for any such device, must reproduce the accompanying copyright notice, this list of conditions and the below disclaimer in capital type, in the DOCUMENTATION and/or other materials provided with the distribution.

3. Neither the name of STMicroelectronics nor the names of other contributors to the SOFTWARE PACKAGE may be used to endorse or promote products derived from the SOFTWARE PACKAGE or part thereof without specific written permission of STMicroelectronics.

4. The SOFTWARE PACKAGE or any part thereof, including modifications and/or derivative works of the SOFTWARE PACKAGE, must be used and execute solely and exclusively on or in combination with a device or a processing unit device manufactured by or for ST, including without limitation processing units which are part of a sensor and/or microcontrollers and/or microprocessors manufactured by or for STMicroelectronics.

5. No use, reproduction or redistribution of the SOFTWARE PACKAGE partially or totally may be done in any manner that would subject the SOFTWARE PACKAGE to any Open Source Terms (as defined below).

6. Some portion of the SOFTWARE PACKAGE may contain software subject to Open Source Terms (as defined below) applicable for each such portion (“Open Source Software”), as further specified in the SOFTWARE PACKAGE. Such Open Source Software is supplied under the applicable Open Source Terms and is not subject to the terms and conditions of this AGREEMENT. “Open Source Terms” shall mean any open source license which requires as part of distribution of software that the source code of such software is distributed therewith or otherwise made available, or open source license that substantially complies with the Open Source definition specified at www.opensource.org and any other comparable open source license such as for example GNU General Public License (GPL), Eclipse Public License (EPL), Apache Software License, BSD license and MIT license.

7. The SOFTWARE PACKAGE may also include third party software as expressly specified in the SOFTWARE PACKAGE subject to specific license terms from such third parties. Such third party software is supplied under such specific license terms and is not subject to the terms and conditions of this AGREEMENT. By installing copying, downloading, accessing or otherwise using the SOFTWARE PACKAGE, the RECIPIENT agrees to be bound by such license terms with regard to such third party software.

8. STMicroelectronics has no obligation to provide any maintenance, support or updates for the SOFTWARE PACKAGE.

9. The SOFTWARE PACKAGE is and will remain the exclusive property of STMicroelectronics and its licensors. The RECIPIENT will not take any action that jeopardizes STMicroelectronics and its licensors' proprietary rights or acquire any rights in the SOFTWARE PACKAGE, except the limited rights specified hereunder.

10.The RECIPIENT shall comply with all applicable laws and regulations affecting the use of the SOFTWARE PACKAGE or any part thereof including any applicable export control law or regulation.

&nbsp;&nbsp;(i) With respect to any SOFTWARE PACKAGE supplied from STMICROELCTRONICS under this AGREEMENT, the RECIPIENT hereby undertakes not to sell, supply, transfer or export, directly or indirectly (including through agents, representatives, or distributors), any SOFTWARE PACKAGE to any third party in the Russian Federation, Belarus or any other sanctioned or restricted countries and/or for intended final use in the Russian Federation, Belarus or any other sanctioned or restricted countries.

&nbsp;&nbsp;(ii) The RECIPIENT shall ensure that a substantially similar provision is included in all its contracts with third parties concerning the SOFTWARE PACKAGE.

&nbsp;&nbsp;(iii) In addition, it is the RECIPIENT’s obligation to conduct the necessary due diligence to ensure screening of RECIPIENT’s end customers and ultimate customers against all applicable restricted or sanctioned party lists, including the applicable EU restricted party lists, the BIS Entity List and the BIS Military End User/Use List.

&nbsp;&nbsp;(iv) The RECIPIENT shall keep complete and accurate records pertaining to the redistribution of SOFTWARE PACKAGE and shall make any such records available for audit by STMICROELECTRONICS or its auditors at any time upon the request of STMICROELECTRONICS, so as to enable STMICROELECTRONICS to check compliance by the RECIPIENT with above sections (i), (ii) and (iii) STMICROELECTRONICS reserves the right to request and review evidence of due diligence and compliance with the above requirements, including the screening of RECIPIENT’s customers.

&nbsp;&nbsp;(v) In case of breach of any of the RECIPIENT commitments under sections (i), (ii), (iii) and/or (iv), STMICROELECTRONICS shall be entitled to immediately terminate this AGREEMENT upon giving written notice to the RECIPIENT with no prejudice to STMICROELECTRONICS’s right to be fully indemnified by RECIPIENT from any liability, losses, damages or costs of any sort incurred or suffered by STMICROELECTRONICS as a result of any such breach.  RECIPIENT also understands that STMICROELECTRONICS is legally obligated to report any such breach to the applicable authorities and consents to such disclosure.

11. Redistribution and use of the SOFTWARE PACKAGE partially or any part thereof other than as permitted under this AGREEMENT is void and will automatically terminate RECIPIENT’s rights under this AGREEMENT.

DISCLAIMER:

THE SOFTWARE PACKAGE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THE SOFTWARE PACKAGE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

EXCEPT AS EXPRESSLY PERMITTED HEREUNDER AND SUBJECT TO THE APPLICABLE LICENSING TERMS FOR ANY THIRD-PARTY SOFTWARE INCORPORATED IN THE SOFTWARE PACKAGE AND OPEN SOURCE TERMS AS APPLICABLE, NO LICENSE OR OTHER RIGHTS, WHETHER EXPRESS OR IMPLIED, ARE GRANTED UNDER ANY PATENT OR OTHER INTELLECTUAL PROPERTY RIGHTS OF STMICROELECTRONICS OR ANY THIRD PARTY.

----------------------------------------

THIS AGREEMENT DOES NOT PROHIBIT THE RECIPIENT FROM USING THE SOFTWARE PACKAGE (MODIFIED OR UNMODIFIED) IN THE DESIGN OF, MANUFACTURE OF, OR USE IN ANY AUTOMOTIVE DEVICE, SYSTEM, APPLICATION, OR ENVIRONMENT (“AUTOMOTIVE USE”), AND THE RECIPIENT ACKNOWLEDGES AND THE PARTIES AGREE THAT (I) THE SOFTWARE PACKAGE, AS PROVIDED BY STMICROELECTRONICS, HAVE NOT BEEN EVALUATED, QUALIFIED, OR OTHERWISE VALIDATED FOR ANY AUTOMOTIVE USE, (II) STMICROELECTRONICS PROVIDES NO REPRESENTATION OR WARRANTY THAT THE SOFTWARE PACKAGE ARE CAPABLE OF BEING SUCCESSFULLY USED OR SUCCESSFULLY QUALIFIED FOR ANY AUTOMOTIVE USE, (III) THE RECIPIENT SHALL TAKE SOLE RESPONSIBILITY FOR ANY AUTOMOTIVE USE, (IV) THE RECIPIENT REPRESENTS, WARRANTS, AND AGREES THAT THE RECIPIENT WILL NOT USE THE SOFTWARE PACKAGE IN ANY AUTOMOTIVE USE WITHOUT FIRST PERFORMING APPROPRIATE EVALUATION, QUALIFICATION, OR OTHER VALIDATION TO SATISFY ALL LAWS, REGULATIONS, SAFETY, AND OTHER REQUIREMENTS, AND (V) IF THE RECIPIENT USES THE SOFTWARE PACKAGE IN ANY AUTOMOTIVE USE, THE RECIPIENT SHALL FULLY INDEMNIFY STMICROELECTRONICS FOR ANY LOSSES, DAMAGES, COSTS (INCLUDING LEGAL FEES) AND EXPENSES INCURRED BY OR FINALLY AWARDED (I.E., NOT APPEALABLE) AGAINST STMICROELECTRONICS AS A RESULT OF OR IN CONNECTION WITH ANY SUCH AUTOMOTIVE USE. THIS PARAGRAPH SETS FORTH ESSENTIAL CONDITIONS OF RECIPIENT’S DECISION TO ENTER INTO THIS AGREEMENT, WITHOUT WHICH RECIPIENTWOULD NOT HAVE ENTERED INTO THIS AGREEMENT.

EXCEPT AS EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING PARAGRAPH, THE RECIPIENT SHALL NOT USE THE SOFTWARE PACKAGE IN ANY SAFETY CRITICAL APPLICATIONS (INCLUDING, WITHOUT LIMITATION, LIFE SUPPORTING DEVICE OR SYSTEMS, MILITARY, NUCLEAR, OR AEROSPACE DEVICES, SYSTEMS, APPLICATIONS OR ENVIRONMENTS). THE RECIPIENT SHALL FULLY INDEMNIFY STMICROELECTRONICS FOR ANY LOSSES, DAMAGES, COSTS (INCLUDING LEGAL FEES) AND EXPENSES INCURRED BY OR FINALLY AWARDED (I.E., NOT APPEALABLE) AGAINST STMICROELECTRONICS AS A RESULT OF OR IN CONNECTION WITH ANY SUCH USE. THIS PARAGRAPH SETS FORTH ESSENTIAL CONDITIONS OF RECIPIENT’S DECISION TO ENTER INTO THIS AGREEMENT, WITHOUT WHICH RECIPIENT WOULD NOT HAVE ENTERED INTO THIS AGREEMENT.

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section4" checked aria-hidden="true">
<label for="collapse-section4" aria-hidden="true">__SLA0078__ – Software License Agreement__</label>
<div>

SLA0078 Rev2/February 2019

## Software license agreement

### __LIMITED LICENSE AGREEMENT FOR ST MATERIALS EVALUATION__

__IMPORTANT-READ CAREFULLY:__

This Limited License Agreement for ST Materials Evaluation (“LLA”) is a legally binding contract between you, on behalf of
either yourself individually or the single entity by which you are employed (referred to in this LLA as "You" or “Licensee”) and
STMicroelectronics International NV, a company incorporated under the laws of the Netherlands acting for the purpose of this
LLA through its Swiss branch located at 39, Chemin du Champ des Filles, 1228 Plan-les-Ouates, Geneva, Switzerland
(hereinafter “ST”).

THE LICENSED MATERIALS ARE LICENSED TO YOU ON THE CONDITION THAT YOU ACCEPT ALL THE TERMS AND
CONDITIONS OF THIS LLA. BY CLICKING ON THE "I ACCEPT" BUTTON OR BY INSTALLING, COPYING, DOWNLOADING,
ACCESSING OR OTHERWISE USING THE LICENSED MATERIALS, YOU AGREE TO BE BOUND BY THE TERMS AND
CONDITIONS OF THIS LLA. IF YOU DO NOT AGREE WITH ANY TERM OR CONDITION OF THIS LLA, DO NOT INSTALL,
ACCESS OR USE THE LICENSED MATERIALS.

For purposes of this LLA:

“Affiliates” shall mean any corporation, partnership, or other entity that, directly or indirectly, owns, is owned by, or is under
common ownership with ST, for so long as such ownership exists. For the purposes of the foregoing, "own", "owned," or
"ownership" shall mean ownership of more than fifty percent (50%) of the stock or other equity interests entitled to vote for the
election of directors or an equivalent governing body.

“Licensed Materials” shall mean the software, hardware, media, information, Documentation, and/or other materials which
accompany this LLA or are otherwise made available by ST and/or its Affiliates under this LLA, any supplements and updates of
any of the foregoing, and any related items or information delivered or made available by ST after formation of this LLA if no
separate agreement is entered into by ST and Licensee to govern such related items or information.

“Documentation” shall mean any comments, annotations, instructions, manuals, guides, and other written materials, whether in
printed or electronic form, which accompany this LLA or are otherwise made available by ST and/or its Affiliates under this LLA.

__APPLICABLE TERMS AND CONDITIONS__

The Licensed Materials contains various components that may be subject to different terms and conditions. The applicable
terms and conditions are as follows:

\(i\) FOR LICENSED MATERIALS THAT CONSTITUTE OPEN SOURCE SOFTWARE: THE APPLICABLE OPEN SOURCE
TERMS, WHICH ARE SET FORTH OR IDENTIFIED IN SOURCE CODE HEADER(S), TEXT FILE(S), OR OTHER
DOCUMENTATION IN OR ACCOMPANYING THIS LLA OR THE LICENSED MATERIALS. “OPEN SOURCE TERMS” MEANS
ANY OPEN SOURCE LICENSE THAT COMPLIES WITH THE OPEN SOURCE DEFINITION SPECIFIED AT
[WWW.OPENSOURCE.ORG](https://opensource.org) AND ANY OTHER COMPARABLE OPEN SOURCE LICENSE SUCH AS FOR EXAMPLE GNU
GENERAL PUBLIC LICENSE (GPL), ECLIPSE PUBLIC LICENSE (EPL), APACHE SOFTWARE LICENSE, BSD LICENSE
AND MIT LICENSE. SUCH OPEN SOURCE SOFTWARE IS NOT SUBJECT TO THE TERMS OF THIS LLA TO THE EXTENT
THE TERMS OF THIS LLA ARE IN CONFLICT WITH THE OPEN SOURCE TERMS APPLICABLE TO SUCH OPEN SOURCE
SOFTWARE. EXCEPT FOR OPEN SOURCE SOFTWARE, YOU HAVE NO RIGHTS UNDER THIS LLA TO, AND MAY NOT
UNDER ANY CIRCUMSTANCES USE ANY LICENSED MATERIALS IN ANY MANNER THAT WOULD, MAKE SUCH
LICENSED MATERIALS SUBJECT TO ANY OPEN SOURCE TERMS. THESE ACTIONS INCLUDE BUT ARE NOT LIMITED
TO COMBINING OPEN SOURCE SOFTWARE WITH THE LICENSED MATERIALS BY MEANS OF INCORPORATION OR
LINKING OR OTHERWISE;

YOU AGREE NOT TO INSTALL OR USE ANY LICENSED MATERIALS THAT CONSTITUTE OPEN SOURCE SOFTWARE.
HOWEVER, THE PROHIBITION OF THE IMMEDIATELY PRECEDING SENTENCE SHALL NOT APPLY IF YOU READ AND
COMPLY WITH THE OPEN SOURCE TERMS APPLICABLE TO THE OPEN SOURCE SOFTWARE.

\(ii\) FOR LICENSED MATERIALS THAT CONSTITUTE THIRD PARTY COMPONENTS: THE APPLICABLE THIRD PARTY
LICENSE TERMS, WHICH ARE SET FORTH OR IDENTIFIED IN SOURCE CODE HEADER(S), TEXT FILE(S), OR OTHER
DOCUMENTATION IN OR ACCOMPANYING THIS LLA OR THE LICENSED MATERIALS (COLLECTIVELY, “THIRD PARTY
TERMS”). SUCH THIRD PARTY COMPONENTS ARE NOT SUBJECT TO THE TERMS OF THIS LLA TO THE EXTENT THE
TERMS OF THIS LLA ARE IN CONFLICT WITH SUCH THIRD PARTY TERMS. LICENSEE ACKNOWLEDGES AND AGREES
THAT THIS LLA DOES NOT CONVEY A LICENSE TO ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY
AND THAT LICENSEE IS RESPONSIBLE FOR ANY FEES OR ROYALTIES PAYABLE TO ANY THIRD PARTY BASED ON
ANY THIRD PARTY’S INTELLECTUAL PROPERTY RIGHTS OR BASED ON ANY THIRD PARTY'S INTERESTS IN ANY
THIRD PARTY COMPONENTS;

YOU AGREE NOT TO INSTALL OR USE ANY LICENSED MATERIALS THAT CONSTITUTE THIRD PARTY COMPONENTS.
HOWEVER, THE PROHIBITION OF THE IMMEDIATELY PRECEDING SENTENCE SHALL BE SUSPENDED IF YOU READ
AND COMPLY WITH THE THIRD PARTY TERMS APPLICABLE TO THE THIRD PARTY COMPONENTS, WHILE SUCH
COMPLIANCE CONTINUES.

\(iii\) FOR ALL OTHER SOFTWARE: THE TERMS AND CONDITIONS OF THIS LLA, WHETHER OR NOT THEY ARE SET
FORTH OR IDENTIFIED IN SOURCE CODE HEADER(S), TEXT FILE(S), OR OTHER DOCUMENTATION IN OR
ACCOMPANYING THIS LLA OR THE LICENSED MATERIALS.

__1. LIMITED LICENSE__

Subject to the terms and conditions of this LLA and any applicable Open Source Terms and Third Party Terms, ST hereby
grants to You, during the term of this LLA, under intellectual property rights owned by ST, a worldwide, non-exclusive, nontransferable, royalty-free license, without the right to sublicense, to use internally the Licensed Materials solely and exclusively
on or in combination with microcontrollers and microprocessors manufactured by or for ST (collectively “ST Chipsets”) for the
Limited License Purpose only. “Limited License Purpose” shall mean the sole purpose of performing evaluation, simulation,
testing, development, and demonstration (but not distribution, sale, license, or other commercialization) of Licensee’s software/
hardware applications solely with ST Chipsets.

To the extent the Licensed Materials are made available to You in source code and subject to the terms and conditions of this
LLA and any applicable Open Source Terms and Third Party Terms, ST hereby grants to You, during the term of this LLA, under
intellectual property rights owned by ST, a worldwide, non-exclusive, non-transferable, royalty-free license, without the right to
sublicense, to use, reproduce and modify (all internally) the source code form of the Licensed Materials and to compile internally
such modified source code into new object code, for the Limited License Purpose only.

Subject to the terms and conditions of this LLA and any applicable Open Source Terms and Third Party Terms, ST hereby
grants to You, during the term of this LLA, under intellectual property rights owned by ST, a worldwide, non-exclusive, nontransferable, royalty-free license, without the right to sublicense, to use internally and make internally a reasonable number of
copies of the Documentation provided by ST under this LLA, solely to support the Limited License Purpose.

You are authorized to have third party contractors exercise on Your behalf the license rights as set forth above, provided that (i)
such contractors are obligated by written agreement with You to maintain the confidentiality of the Licensed Materials to at least
the degree required of You under this LLA, (ii) You shall ensure such contractors comply with the terms of this LLA, (iii) You shall
be liable for any breach by such contractors, and (iv) You shall indemnify and hold harmless ST and its Affiliates for any harm
resulting from such contractor’s use of the Licensed Materials.

You acknowledge that the Licensed Materials have not been specifically designed to meet Your individual requirements and that
You have all information necessary to evaluate whether the Licensed Materials meet Your requirements or not, and will be
suitable for Your intended use or application. Therefore the Licensed Materials shall be deemed accepted upon delivery to
Licensee. You shall use, at Your own risk, the Licensed Materials and any development that is obtained from such use. You
acknowledge that ST and its Affiliates cannot in any way be held responsible for the consequences resulting from use of the
Licensed Materials and any development made following such use of the Licensed Materials, and You shall indemnify and hold
harmless ST and its Affiliates for any harm resulting from Your use of the Licensed Materials or Your software/hardware
applications in any manner not permitted under this LLA.

You acknowledge that You are receiving the Licensed Materials under a limited license and not as a purchaser of the Licensed
Materials.

__2. RESTRICTIONS__

Unless otherwise expressly stipulated under Article 1, You shall not, and shall not permit any third party to: (i) copy, reproduce or
duplicate the Licensed Materials; (ii) translate, modify, adapt, decompile, disassemble or reverse engineer and make derivative
works of (any portion of) the Licensed Materials; (iii) rent, disclose, publish, sell, assign, lease, lend, sublicense, market,
transfer, distribute or otherwise provide third parties access to (any portion of) the Licensed Materials for any purpose; (iv)
attempt to derive the source code, algorithmic nature or structure of any object code portions of the Licensed Materials; (v) use
the Licensed Materials to create any product that competes with the Licensed Materials or ST Chipsets; (vi) remove or
circumvent any protection or other restrictive technology mechanism of the Licensed Materials or ST Chipsets; (vii) disclose the
results of the Licensed Materials’ performance to any third party; or (viii) otherwise use any portion of the Licensed Materials in
any manner not expressly authorized by this LLA.

Other than the limited licenses expressly granted to You under Article 1 herein, no other rights or licenses are granted, or
implied by estoppel or otherwise, under any intellectual property rights of ST and/or its Affiliates or any intellectual property of a
third-party residing in the Licensed Materials or in any other confidential information furnished by ST and its Affiliates, including,
without limitation, (i) for the combination of such Licensed Materials or other confidential information with one or more other
items (including items acquired from ST and/or its Affiliates) even if such items have no substantial use other than as part of
such combination (ii) with respect to any trademark, trade or brand name, domain name, corporate name of ST and/or Affiliates,
or any other name or mark, or contraction abbreviation or simulation thereof, (iii) under any intellectual property rights covering
any standard set by a standard setting body and any de facto standards.

You may not use the Licensed Materials for commercial purposes nor sell, or otherwise transfer it for value. Commercial
purposes include the use of the Licensed Materials as a standalone product or in a product, to be distributed, marketed or
otherwise offered for sale or for rent.

Notwithstanding any other terms of this LLA, You further acknowledge that the Licensed Materials are not specifically designed
for use in safety critical applications such as life supporting devices or systems and nuclear, automotive or aerospace
applications or environments. ST, its Affiliates, and/or its licensors expressly disclaim any responsibility for such usage which
shall be made by You at your sole risks, even if ST, its Affiliates, and/or its licensors have been informed by You in writing of
such usage. You acknowledge and agree that You shall be solely responsible for regulatory, safety and security related
requirements concerning your own products and use of Licensed Materials in your products and related applications.

You shall limit access and use of the Licensed Materials to You and those individuals employed by You who have a need to
access the Licensed Materials for the Limited License Purpose. You agree that, (i) such individuals shall be obligated by written
agreement with You to maintain the confidentiality of the Licensed Materials to at least the degree required of You under this
LLA, (ii) You shall ensure such individuals comply with the terms of this LLA, (iii) You shall be liable for any breach by such
individuals, and (iv) You shall indemnify and hold harmless ST and its Affiliates for any harm resulting from such individual’s use
of the Licensed Materials. Except as expressly permitted under Article 1, or as otherwise expressly agreed in writing by ST, You
shall not allow any third party to use the Licensed Materials.

ST and its Affiliates have no obligation to provide You with maintenance, technical support or updates for the Licensed
Materials.

__3. OWNERSHIP, COPYRIGHTS AND TRADEMARKS__

The Licensed Materials are and shall remain exclusively owned by ST, its Affiliates, and/or its licensors, whether or not
specifically recognized or perfected under the laws of the country where the Licensed Materials originate or are used. You shall
not take any action that jeopardizes ST’s, its Affiliates’, and/or its licensors' ownership of the Licensed Materials. You shall not
acquire any rights in the Licensed Materials, except the limited rights specified in Article 1.

You shall ensure that all notices, including but not limited to all copyright and trademark notices of ST, its Affiliates, and its
licensors are reproduced in any copy of the whole or any part of the Licensed Materials. You shall not remove, modify or alter
any such notice from any part of the Licensed Materials.

__4. CONFIDENTIALITY__

Confidential information shall include (i) all information provided by ST or its Affiliates in relation to the Licensed Materials which
(a) is designated as “confidential”, “proprietary” or with a similar legend, and (b) is by its own nature of a type which would
reasonably be considered confidential and (ii) the Licensed Material (“Confidential Information”). You may only use the
Confidential Information for the Limited License Purpose and shall protect the confidentiality of the Confidential Information by
using the same degree of care, but not less than a reasonable degree of care, to prevent its unauthorized use, dissemination, or
publication as You use to protect your own confidential information of like nature.

You shall be liable towards ST for any damages incurred by ST or its Affiliates due to a failure by You or Your employees to
comply with the provisions in this LLA. You shall immediately notify ST of any unauthorised use or disclosure of, or of any
unauthorised access to, or of any theft or loss of, the Licensed Materials or Confidential Information or part thereof, which
comes to Your notice.

The confidentiality obligations shall not apply to Confidential Information, which can be shown by documentary evidence: (i)
entered the public domain through no fault of the Licensee; or, (ii) was known to the Licensee prior to receipt from ST and its
Affiliates; or, (iii) was disclosed to the Licensee by a third party having the right to disclose; or, (iv) was independently developed
by the Licensee without use of the Licensed Materials or Confidential Information, the burden of proof of independence being on
the Licensee.

These confidentiality obligations shall survive any termination or expiration of the LLA for whatever cause.

__5. DISCLAIMER OF WARRANTY__

YOU ACKNOWLEDGE THAT THE LICENSED MATERIALS AND ANY AND ALL PARTS THEREOF ARE PROVIDED “AS IS”.
ST AND ITS AFFILIATES MAKE NO REPRESENTATIONS OR WARRANTIES WITH RESPECT TO THE LICENSED
MATERIALS AND OR PARTS THEREOF WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. SPECIFICALLY,
WITHOUT LIMITING THE GENERALITY OF THE FOREGOING, ST AND ITS AFFILIATES MAKE NO REPRESENTATION OR
WARRANTY THAT (I) THE USE OF THE LICENSED MATERIALS AND OR PARTS THEREOF WILL BE UNINTERRUPTED
OR ERROR FREE, OR (II) ANY USE OF THE LICENSED MATERIALS AND OR PARTS THEREOF, WHETHER OR NOT
INTEGRATED INTO OTHER SOFTWARE OR EQUIPMENT, ARE FREE FROM INFRINGEMENT OF ANY THIRD PARTY
INTELLECTUAL PROPERTY RIGHTS. IT SHALL BE YOUR SOLE RESPONSIBILITY TO MAKE SUCH DETERMINATION AS
IS NECESSARY WITH RESPECT TO THE ACQUISITION OF LICENSES UNDER PATENTS AND OTHER INTELLECTUAL
PROPERTY RIGHTS OF THIRD PARTIES. CONSEQUENTLY, ST AND ITS AFFILIATES DISCLAIM ANY LIABILITY IN CASE
ANY USE BY YOU OR YOUR CONTRACTORS INFRINGES ANY THIRD PARTY’S INTELLECTUAL PROPERTY RIGHTS.
FURTHERMORE ST AND ITS AFFILIATES HEREBY DISCLAIM ANY WARRANTY AND LIABILITY WHATSOEVER FOR ANY
DEVELOPMENT CREATED BY OR FOR YOU OR YOUR CUSTOMERS.

YOU ACKNOWLEDGE THAT YOU HAVE RELIED ON NO WARRANTIES OTHER THAN THE EXPRESS WARRANTIES IN
THIS LLA AND THAT NO WARRANTIES ARE MADE BY ST OR ITS AFFILIATES OR GRANTED BY LAW WHENEVER IT IS
PERMITTED BY LAW.

__6. COMPLIANCE__

You agree not to use the Licensed Materials in violation of any law, statute, ordinance or other regulation or any obligation by
which You are bound. You agree to comply with all applicable laws and regulations regarding the use of the Licensed Materials.
Specifically but without limiting the generality of the foregoing, You acknowledge that the Licensed Materials are subject to
export controls restrictions and You agree to comply with the European export regulations and US similar regulations, and to
obtain any necessary export license or other documentation prior to exportation or re-exportation of the Licensed Materials.

Some Licensed Materials may (i) require licenses from third parties claiming intellectual property rights covering use or
implementation of the Licensed Materials or (ii) be based on industry recognized standards or software programs published by
industry recognized standards bodies and certain third parties may claim to own intellectual property rights that cover
implementation or use of those standards. You agree that You will obtain any necessary licenses from such third parties prior to
execution of the license rights granted by ST to You herein.

__7. DISCLAIMER OF DAMAGES__

IN NO EVENT SHALL ST AND ITS AFFILIATES BE LIABLE TO LICENSEE OR ANY THIRD PARTY FOR ANY INDIRECT,
INCIDENTAL, PUNITIVE, SPECIAL OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST SAVINGS, LOSS OF
BARGAIN OR OPPORTUNITY, PROFESSIONAL FEES OR EXPENSES, BUSINESS INTERRUPTION, LOST REVENUES OR
SALES, DAMAGE TO PRODUCT OR EQUIPMENT OR TO FACILITIES, COSTS OF SUBSTITUTE PRODUCT, FACILITIES
OR SERVICES, REWORK CHARGES, COSTS ASSOCIATED WITH DOWN TIME, LOSS OF GOODWILL, LOSS OF DATA
OR FOR ANY DAMAGES COSTS OR EXPENSES ASSOCIATED WITH WARRANTY OR INTELLECTUAL PROPERTY
INFRINGEMENT CLAIMS, WHETHER FORESEEABLE OR UNFORESEEABLE AND WHETHER OR NOT SUCH DAMAGES
ARE BASED ON WARRANTY, CONTRACT OR ANY OTHER LEGAL THEORY, ARISING OUT OF OR IN CONNECTION
WITH THIS LLA – EVEN IF LICENSEE HAS BEEN ADVISED, OR IS AWARE, OF THE POSSIBILITY OF SUCH DAMAGES.
ST AND ITS AFFILIATES AGGREGATE AND CUMULATIVE LIABILITY TOWARDS LICENSEE UNDER THIS LLA SHALL NOT
EXCEED 100 USD (ONE HUNDRED US DOLLARS). THE LIMITATIONS SET FORTH ABOVE IN THIS ARTICLE 7 SHALL
ONLY APPLY TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW.

__8. TERM AND TERMINATION__

This LLA shall be valid for a period that starts upon Your acceptance of the terms and conditions of this LLA and ends upon the
earlier of two (2) years or ST’s making the Licensed Materials or a subsequent version of the Licensed Materials available for
download via [www.st.com](https://www.st.com/). Licensee may terminate this LLA at any time by destroying all Licensed Materials and other
Confidential Information or by returning those to ST. ST may terminate this LLA at any time if (i) You fail to comply with the terms
and conditions of this LLA, (ii) You threaten or file litigation against ST or its Affiliates, (iii) ST or its Affiliates receive notice of any
claim, suit or proceeding that alleges that the Licensed Materials or Your use of the Licensed Materials infringes any third-party
intellectual property rights, or (iv) to the extent permitted by laws, a voluntary or involuntary petition in bankruptcy or winding up
is filed against Licensee, any proceedings in insolvency or bankruptcy are instituted against Licensee, a trustee or receiver is
appointed over Licensee, or any assignment is made for the benefit of creditors of Licensee.

Upon termination or expiration, You shall delete, destroy, or return to ST all Licensed Materials in Your possession and You shall
stop using the Licensed Materials for any purpose whatsoever.

Articles 2 to 10, inclusive, shall survive the termination or expiration of this LLA.

__9. MISCELLANEOUS__

If a court or agency of competent jurisdiction holds any term of this LLA invalid, illegal, or unenforceable for any reason, the
remainder of this LLA shall be valid and enforceable and You and ST shall discuss in good faith a substitute, valid, enforceable
provision which most nearly effects the parties intent in entering into this LLA.

The delay or failure by ST to enforce any provisions of this LLA or to exercise any right in respect thereto shall not be construed
as constituting a waiver of any right of ST or obligation of Licensee.

No agency, joint venture, partnership or other business organization shall be created or be construed as being created by
reason of this LLA. You will not have the right or authority to, and shall not, assume or create any obligation of any nature
whatsoever on behalf of ST and its Affiliates or bind ST and its Affiliates in any respect whatsoever. You and ST are independent
parties. Nothing in this LLA shall be construed as making You an employee, agent or legal representative of ST.

This LLA contains the entire and sole agreement between You and ST on the subject matter of this LLA, and supersedes all
representations, undertakings and agreements previously made between You and ST and/or its Affiliates and shall prevail over
the terms and conditions set forth in any document from You with respect to the subject matter of this LLA. Any amendment to
this LLA shall be agreed in writing and be duly signed by You and ST and shall make clear reference to this LLA.

Licensee agrees that ST’s Affiliates are intended to be third party beneficiaries of this LLA.

Licensee agrees that in the event of any breach of this LLA by Licensee or its contractors, the resulting harm to ST and its
Affiliates would likely be of a nature that monetary damages would not provide an adequate remedy, and therefore, ST and its
Affiliates shall be entitled to seek specific performance, injunction, and other equitable relief.

__10. APPLICABLE LAW__

This LLA is governed by the laws of Switzerland, without regard to its conflict of law rules. All disputes or questions arising out of
or in connection with this LLA shall be finally settled by the competent courts of Geneva, Switzerland. Licensee hereby waives
any objection to venue. Notwithstanding the aforesaid, nothing in this LLA shall prevent ST from seeking any interim or final
injunctive or equitable relief by a court of competent jurisdiction.

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section2" checked aria-hidden="true">
<label for="collapse-section2" aria-hidden="true">__ANNEX 1: MICROSOFT SOFTWARE LICENSE TERMS__</label>
<div>

MICROSOFT AZURE RTOS

Shape

These license terms are an agreement between you and Microsoft Corporation (or
one of its affiliates). They apply to the software named above and any Microsoft
services or software updates (except to the extent such services or updates are
accompanied by new or additional terms, in which case those different terms
apply prospectively and do not alter your or Microsoft’s rights relating to
pre-updated software or services). IF YOU COMPLY WITH THESE LICENSE TERMS, YOU
HAVE THE RIGHTS BELOW.  BY USING THE SOFTWARE, YOU ACCEPT THESE TERMS.

1. INSTALLATION AND USE RIGHTS.

a) General. You may install and use the software and the included Microsoft
applications solely for internal development, testing and evaluation purposes.
Any distribution or production use requires a separate license as set forth in
Section 2.

b) Contributions. Microsoft welcomes contributions to this software. In the event
that you make a contribution to this software you will be required to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and
actually do, grant Microsoft the rights to use your contribution. For details,
visit https://cla.microsoft.com.

c) Included Microsoft Applications. The software includes other Microsoft
applications which are governed by the licenses embedded in or made available
with those applications.

d) Third Party Components. The software may include third party components with
separate legal notices or governed by other agreements, as may be described
within the software or in the ThirdPartyNotices file(s) accompanying the
software.

e) Competitive Benchmarking. If you are a direct competitor, and you access or use
the software for purposes of competitive benchmarking, analysis, or intelligence
gathering, you waive as against Microsoft, its subsidiaries, and its affiliated
companies (including prospectively) any competitive use, access, and
benchmarking test restrictions in the terms governing your software to the
extent your terms of use are, or purport to be, more restrictive than
Microsoft’s terms. If you do not waive any such purported restrictions in the
terms governing your software, you are not allowed to access or use this
software, and will not do so.

2. DISTRIBUTION AND PRODUCTION USE. If you have obtained and/or are developing on
microprocessor(s) and/or microcontroller(s) (“hardware”) listed in the file
named “LICENSED-HARDWARE.txt” included in the repository and/or distributed with
the software you have the following rights in and to the software solely when
used in combination with the hardware. In the event hardware is not listed in
the LICENSED-HARDWARE.txt file, you do not have the rights in this Section 2.

a) Distribution and Production Use Rights.

i. You may use the software in production (e.g. program the modified or unmodified
software to devices you own or control) and distribute (i.e. make available to
third parties) the modified or unmodified binary image produced from this code.


ii. You may permit your device distributors or developers to copy and distribute the
binary image as programmed or to be programmed to your devices.

iii. You may redistribute the unmodified or modified source to your device
distributors or developers. Modifications must be clearly marked. Any
redistribution in source code form must contain this license and any other
licenses that accompany the software.

b) Requirements. For any code you distribute, you must:

i. when distributed in binary form, except as embedded in a device, include with
such distribution the terms of this agreement;

ii. when distributed in source code form to distributors or developers of your
devices, include with such distribution the terms of this agreement; and

iii. indemnify, defend and hold harmless Microsoft from  any claims, including
attorneys’ fees, related to the distribution or use of your devices, except to
the extent that any claim is based solely on the unmodified software.

c) Restrictions. You may not:

i. use or modify the software to create a competing real time operating system
software;

ii. remove any copyright notices or licenses contained in the software;

iii. use Microsoft’s trademarks or trade dress in your application in any way that
suggests your device or application comes from or is endorsed by Microsoft;

iv. transfer individual components, specific libraries, classes, functions or code
fragments of the software separately for purposes unrelated to the software; or

v. use or distribute the software in any way that would subject the software or
Microsoft’s intellectual property or technology to any other license terms.

3. SCOPE OF LICENSE. The software is licensed, not sold. Microsoft reserves all
other rights. Unless applicable law gives you more rights despite this
limitation, you will not (and have no right to):

a) remove, minimize, block, or modify any notices of Microsoft or its suppliers in
the software;

b) use the software in any way that is against the law or to create or propagate
malware; or

c) share, publish, distribute, or lease the software (except as permitted in
Section 2 above), or provide the software as a stand-alone offering for others
to use.

4. DATA. This software may interact with other Microsoft products that collect data
that is transmitted to Microsoft. To learn more about how Microsoft processes
personal data we collect, please see the Microsoft Privacy Statement at
https://go.microsoft.com/fwlink/?LinkId=248681.

5. EXPORT RESTRICTIONS. You must comply with all domestic and international export
laws and regulations that apply to the software, which include restrictions on
destinations, end users, and end use. For further information on export
restrictions, visit https://aka.ms/exporting.

6. SUPPORT SERVICES. Microsoft is not obligated under this agreement to provide any
support services for the software. Any support provided is “as is”, “with all
faults”, and without warranty of any kind.

7. UPDATES. Microsoft may periodically update the software. You may obtain updates
only from Microsoft or Microsoft-authorized sources. Updates may not include or
support all existing software features, services, or peripheral devices.

8. TERMINATION. Without prejudice to any other rights, Microsoft may terminate this
agreement if you fail to comply with any of its terms or conditions. In such
event, you must destroy all copies of the software and all of its component
parts.

9. ENTIRE AGREEMENT. This agreement, and any other terms Microsoft may provide for
supplements, updates, or third-party applications, is the entire agreement for
the software. To the extent you have entered into a separate agreement with
Microsoft relating specifically to the software, the terms in such agreement
shall control.

10. APPLICABLE LAW AND PLACE TO RESOLVE DISPUTES. If you acquired the software in
the United States or Canada, the laws of the state or province where you live
(or, if a business, where your principal place of business is located) govern
the interpretation of this agreement, claims for its breach, and all other
claims (including consumer protection, unfair competition, and tort claims),
regardless of conflict of laws principles. If you acquired the software in any
other country, its laws apply. If U.S. federal jurisdiction exists, you and
Microsoft consent to exclusive jurisdiction and venue in the federal court in
King County, Washington for all disputes heard in court. If not, you and
Microsoft consent to exclusive jurisdiction and venue in the Superior Court of
King County, Washington for all disputes heard in court.

11. CONSUMER RIGHTS; REGIONAL VARIATIONS. This agreement describes certain legal
rights. You may have other rights, including consumer rights, under the laws of
your state or country. Separate and apart from your relationship with Microsoft,
you may also have rights with respect to the party from which you acquired the
software. This agreement does not change those other rights if the laws of your
state or country do not permit it to do so. For example, if you acquired the
software in one of the below regions, or mandatory country law applies, then the
following provisions apply to you:

a) Australia. You have statutory guarantees under the Australian Consumer Law and
nothing in this agreement is intended to affect those rights.

b) Germany and Austria.

i. Warranty. The properly licensed software will perform substantially as
described in any Microsoft materials that accompany the software. However,
Microsoft gives no contractual guarantee in relation to the licensed software.

ii. Limitation of Liability. In case of intentional conduct, gross negligence,
claims based on the Product Liability Act, as well as, in case of death or
personal or physical injury, Microsoft is liable according to the statutory law.


Subject to the foregoing clause ii., Microsoft will only be liable for slight
negligence if Microsoft is in breach of such material contractual obligations,
the fulfillment of which facilitate the due performance of this agreement, the
breach of which would endanger the purpose of this agreement and the compliance
with which a party may constantly trust in (so-called "cardinal obligations").
In other cases of slight negligence, Microsoft will not be liable for slight
negligence.

12. DISCLAIMER OF WARRANTY. THE SOFTWARE IS LICENSED “AS IS.” YOU BEAR THE RISK OF
USING IT. MICROSOFT GIVES NO EXPRESS WARRANTIES, GUARANTEES, OR CONDITIONS. TO
THE EXTENT PERMITTED UNDER APPLICABLE LAWS, MICROSOFT EXCLUDES ALL IMPLIED
WARRANTIES, INCLUDING MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT.

13. LIMITATION ON AND EXCLUSION OF DAMAGES. IF YOU HAVE ANY BASIS FOR RECOVERING
DAMAGES DESPITE THE PRECEDING DISCLAIMER OF WARRANTY, YOU CAN RECOVER FROM
MICROSOFT AND ITS SUPPLIERS ONLY DIRECT DAMAGES UP TO U.S. $5.00. YOU CANNOT
RECOVER ANY OTHER DAMAGES, INCLUDING CONSEQUENTIAL, LOST PROFITS, SPECIAL,
INDIRECT, OR INCIDENTAL DAMAGES.

This limitation applies to (a) anything related to the software, services,
content (including code) on third party Internet sites, or third party
applications; and (b) claims for breach of contract, warranty, guarantee, or
condition; strict liability, negligence, or other tort; or any other claim; in
each case to the extent permitted by applicable law.

It also applies even if Microsoft knew or should have known about the
possibility of the damages. The above limitation or exclusion may not apply to
you because your state, province, or country may not allow the exclusion or
limitation of incidental, consequential, or other damages.



Please note: As this software is distributed in Canada, some of the clauses in
this agreement are provided below in French.

Remarque: Ce logiciel étant distribué au Canada, certaines des clauses dans ce
contrat sont fournies ci-dessous en français.

EXONÉRATION DE GARANTIE. Le logiciel visé par une licence est offert « tel quel
». Toute utilisation de ce logiciel est à votre seule risque et péril. Microsoft
n’accorde aucune autre garantie expresse. Vous pouvez bénéficier de droits
additionnels en vertu du droit local sur la protection des consommateurs, que ce
contrat ne peut modifier. La ou elles sont permises par le droit locale, les
garanties implicites de qualité marchande, d’adéquation à un usage particulier
et d’absence de contrefaçon sont exclues.

LIMITATION DES DOMMAGES-INTÉRÊTS ET EXCLUSION DE RESPONSABILITÉ POUR LES
DOMMAGES. Vous pouvez obtenir de Microsoft et de ses fournisseurs une
indemnisation en cas de dommages directs uniquement à hauteur de 5,00 $ US. Vous
ne pouvez prétendre à aucune indemnisation pour les autres dommages, y compris
les dommages spéciaux, indirects ou accessoires et pertes de bénéfices.

Cette limitation concerne:

•tout ce qui est relié au logiciel, aux services ou au contenu (y compris le
code) figurant sur des sites Internet tiers ou dans des programmes tiers; et

•les réclamations au titre de violation de contrat ou de garantie, ou au titre
de responsabilité stricte, de négligence ou d’une autre faute dans la limite
autorisée par la loi en vigueur.

Elle s’applique également, même si Microsoft connaissait ou devrait connaître
l’éventualité d’un tel dommage. Si votre pays n’autorise pas l’exclusion ou la
limitation de responsabilité pour les dommages indirects, accessoires ou de
quelque nature que ce soit, il se peut que la limitation ou l’exclusion
ci-dessus ne s’appliquera pas à votre égard.

EFFET JURIDIQUE. Le présent contrat décrit certains droits juridiques. Vous
pourriez avoir d’autres droits prévus par les lois de votre pays. Le présent
contrat ne modifie pas les droits que vous confèrent les lois de votre pays si
celles-ci ne le permettent pas.

</div>

:::

:::
:::